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

## Step Execution Model

File: `simulator/planner/step.py`

- `Step.act()` calls `execute()` (once, on first call) then `check()` on each subsequent call.
- `exec_fn()` runs once; `check_fn()` is polled until it returns `True`.
- Blocking loops inside `check_fn()` are acceptable (see `SendMissionCount.check_fn()`).
- `exec_fn()` can also run the full protocol synchronously (see `PushMissionToGCS`), with `check_fn()` returning `True` immediately.

> **Confidence:** Confirmed from repo (`simulator/planner/step.py:136-155`)
