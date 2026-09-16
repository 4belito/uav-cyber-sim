# Mission Planning: AutoPlan, Upload, and QGC Display

## AutoPlan Execution Order

File: `simulator/planner/plans/auto.py`

The **correct** action order is:

```
Plan.arm()          ← pre_arm + set GUIDED mode + arm (takes 10-30 s; QGC downloads empty mission here)
make_upload_mission ← uploads to ArduPilot AFTER QGC's InitialConnectStateMachine finishes
make_start_mission  ← switches to AUTO mode
make_monitoring     ← waits for mission completion
```

**Do not move `make_upload_mission` before `Plan.arm()`.**

Reason: QGC's `InitialConnectStateMachine` downloads parameters (~10–30 s) and then the mission. If `MISSION_COUNT` (upload) collides with QGC's `MISSION_REQUEST_LIST` (download), ArduPilot sets `receiving=true`, returns `MAV_MISSION_DENIED` to QGC, and QGC never retries — mission never appears in QGC.

`Plan.arm()` (specifically `make_pre_arm()`) takes 10–30 s waiting for EKF, which is enough buffer for QGC to finish its initial download.

> **Confidence:** Confirmed from current chat

## Mission Upload Protocol (`simulator/planner/actions/upload_mission.py`)

### Key Design Decisions

**`from_scratch=False` (default)**:
- Skips `ClearMission` step.
- `ClearMission` sends `MISSION_CLEAR_ALL`, which causes ArduPilot to broadcast `MISSION_CURRENT(total=0)` to all links → QGC re-downloads (empty) → race with our upload.
- Only use `from_scratch=True` when explicitly starting fresh.

**`_MissionCache`**:
- Lazy-loads the mission file once from disk; shared across all `SendMissionItem` steps.
- Reduces upload time from N file reads to 1.

**DENIED handling in `SendMissionCount.check_fn()`**:
- If ArduPilot returns `MAV_MISSION_DENIED`, another client (e.g., QGC) holds the upload lock.
- The step backs off 1 s and retries `MISSION_COUNT`.

**`PushMissionToGCS` step (added at end of upload action)**:
- After upload to ArduPilot completes, Logic opens a new UDP socket to `127.0.0.1:14550` (QGC's port).
- Sends `MISSION_COUNT(N)` with `source_system=ArduPilot_sysid` — QGC thinks ArduPilot is pushing.
- QGC responds with `MISSION_REQUEST_INT` to all known UDP hosts (including Logic's new socket AND ArduPilot's socket).
- ArduPilot (not in `receiving` mode) responds directly with `MISSION_ITEM_INT` from its mission store.
- QGC assembles the mission and displays waypoints.
- Controlled by `push_to_gcs=True` parameter on `make_upload_mission()`.

> **Confidence:** Confirmed from current chat

### Step Sequence in `make_upload_mission()`

```
SendMissionCount    ← sends MISSION_COUNT(N); waits for MISSION_REQUEST(seq=0)
SendMissionItem[0]  ← sends item 0; waits for MISSION_REQUEST(seq=1)
SendMissionItem[1]  ← ...
...
SendMissionItem[N-1]← last item; waits for MISSION_ACK(ACCEPTED)
PushMissionToGCS    ← (if push_to_gcs=True) triggers QGC display
```

> **Confidence:** Confirmed from repo (`simulator/planner/actions/upload_mission.py`)

## `Plan.arm()` Composition

File: `simulator/planner/plan.py`

```python
Plan.arm() returns:
  make_pre_arm(firmware=...)   # waits for EKF health (10-30 s)
  make_set_mode(GUIDED)        # switches to GUIDED mode
  [make_change_nav_speed(...)] # optional, if speed != 5.0
  make_arm()                   # arms the vehicle
```

> **Confidence:** Confirmed from repo (`simulator/planner/plan.py:118-141`)

## GuidedPlan: ArduPlane Implementation

File: `simulator/planner/plans/guided.py`

### Takeoff — `PlaneTakeOff` (`takeoff.py`)

```
exec_fn():
  PARAM_SET AUTOLAND_DIR_OFF = (approach_heading - origin_heading) % 360  ← must be before mode switch
  PARAM_SET TKOFF_ALT = altitude
  set_mode(PlaneMode.TAKEOFF)   ← runway roll + climb, no mission upload
check_fn():
  wait for landed_state == IN_AIR
  set_mode(PlaneMode.GUIDED)    ← switch back for DO_REPOSITION navigation
```

- `approach_heading` is computed in `GuidedPlan.__init__` from the last trajectory leg: `math.degrees(math.atan2(dx, dy)) % 360` (ENU frame: `atan2(East, North)` = compass bearing). Stored as `autoland_dir_off` in the plan spec.
- `AUTOLAND_DIR_OFF` **must** be set before ground speed exceeds 5 m/s in TAKEOFF mode — ArduPlane captures `initial_direction = gps.ground_course() + AUTOLAND_DIR_OFF` exactly once. After takeoff the stored direction is frozen.
- `origin_heading` (= SITL spawn heading = `spawn_heading` in logic config) is threaded through `sim.py → spawn_heading → logic.py → VehicleLogic → plan.bind(origin_heading=...) → Step.origin_heading`.

### Navigation — `PlaneGoTo` (`navigation.py`)

Uses `COMMAND_INT` with `Cmd.DO_REPOSITION` (192) — the only command ArduPlane processes for lat/lon in GUIDED mode. `SET_POSITION_TARGET_GLOBAL_INT` is ignored for lat/lon by fixed-wing.

- `wp_margin` default: 90 m (must exceed `WP_LOITER_RAD = 80 m`; plane loiters at that radius and never gets closer to the center).
- `nav_wps = wps[1:-1]` — skips home (first WP) and last WP (landing target) to avoid erratic first-step and redundant final navigation.

### Landing — `PlaneLand` (`land.py`)

```
exec_fn():
  COMMAND_INT DO_SET_HOME → relocates HOME to land_wp position (AUTOLAND targets HOME)
  PARAM_SET AUTOLAND_WP_DIST  (if autoland_wp_dist is not None)
  PARAM_SET AUTOLAND_WP_ALT   (if approach_alt is not None)
  set_mode(PlaneMode.AUTOLAND)
check_fn():
  poll EXTENDED_SYS_STATE until landed_state == ON_GROUND
```

- `approach_alt: float | None = None` — `None` uses `AUTOLAND_WP_ALT` from parm file (recommended).
- `autoland_wp_dist: float | None = None` — `None` uses `AUTOLAND_WP_DIST` from parm file (recommended).
- Parm file values for Zephyr: `AUTOLAND_WP_DIST=100`, `AUTOLAND_WP_ALT=14` (both `gazebo-zephyr.parm` and `plane-zephyr.parm`).
- AUTOLAND always generates a base leg (traffic pattern, 90° turn before final) — cannot be removed without AUTO mode. Best possible footprint with current mode.

### `origin_heading` Threading

`GuidedPlan` needs the SITL spawn heading at step execution time to compute `AUTOLAND_DIR_OFF`. The chain:

1. `sim.py`: parses `spawn_heading = float(visualizer.home_str(veh).split(",")[3])` — actual SITL `--home` heading (visualizer-dependent, see below)
2. Saves as `"spawn_heading"` in `logic_config.json`
3. `logic.py`: reads `config.get("spawn_heading", 0.0)` → passes to `VehicleLogic(spawn_heading=...)`
4. `VehicleLogic.__init__`: `plan.bind(gra_origin, mav_manager, spawn_heading)`
5. `Action.bind` / `Step.bind`: stores `self.origin_heading`
6. `PlaneTakeOff.exec_fn()`: `autoland_param = (self._autoland_dir_off - self.origin_heading) % 360`

**`spawn_heading` per visualizer:**
- **QGC**: `visveh.home.to_str()` → `spawn_heading = vehicle.home.heading`
- **NoVis**: `gra_origin.to_abs(vehicle.home).to_str()` → `spawn_heading = (gra_origin.heading + vehicle.home.heading) % 360`
- **Gazebo**: `gra_origin.unpose().pose(vehicle.home.heading).to_str()` → `spawn_heading = vehicle.home.heading`

Gazebo note: `--home lat/lon` must be `gra_origin` (not `gra_home`) because ArduPilot treats `--home` as the GPS position of world (0,0,0), and the Gazebo model spawns at `(veh.home.x, veh.home.y)`. The heading in `--home` is not used by Gazebo (ignored by the physics bridge), but `spawn_heading` in logic config must equal the Gazebo model compass heading = `vehicle.home.heading` (since `heading_to_yaw(H) = -radians(H)` with North-facing default model → actual compass heading = H).

`self.origin` (type `GRA`) is kept headingless. Heading is stored separately as `self.origin_heading: float`.

> **Confidence:** Confirmed from source and ArduPilot firmware analysis (`mode_autoland.cpp`, `check_takeoff_direction()`)

---

## Step Execution Model

File: `simulator/planner/step.py`

- `Step.act()` calls `execute()` (once, on first call) then `check()` on each subsequent call.
- `exec_fn()` runs once; `check_fn()` is polled until it returns `True`.
- Blocking loops inside `check_fn()` are acceptable (see `SendMissionCount.check_fn()`).
- `exec_fn()` can also run the full protocol synchronously (see `PushMissionToGCS`), with `check_fn()` returning `True` immediately.

> **Confidence:** Confirmed from repo (`simulator/planner/step.py:136-155`)
