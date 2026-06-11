# Known Issues, Root Causes, and Fixes

## [FIXED] Gazebo Plane Renders White (No Texture)

**Symptom:** Zephyr plane spawns in Gazebo with a solid white mesh — color texture not applied.

**Root cause:** `_generate_vehicle_models_from_bases` used a regex to replace the `<include>` URI:
```python
re.sub(r"<include>\s*<uri>model://[^<]+</uri>\s*</include>", ...)
```
This silently fails when `<include>` contains a sibling `<pose>` element (as the zephyr does):
```xml
<include>
  <uri>model://gazebo-zephyr/physics</uri>
  <pose>0 0 0.2 0 0 0</pose>   ← breaks the regex
</include>
```
No match → URI stays as `physics` → Gazebo loads the uncolored physics model → white plane.

**Fix:** Replace regex with targeted string replace (only touches the URI line, leaves `<pose>` intact):
```python
sdf = sdf.replace(
    f"<uri>model://{veh.model}/physics</uri>",
    f"<uri>model://{veh.model}/{veh.color.value}</uri>",
)
```

> **Confidence:** Confirmed; fix implemented in `simulator/visualizer/gazebo/gazebo.py`

---

## [FIXED] QGC Mission Waypoints Never Display

**Symptom:** QGC connects and shows the vehicle, but the mission waypoints never appear in Fly View after the Logic process uploads the mission.

**Root cause:** QGC downloads the mission (empty) during `InitialConnectStateMachine` startup, before Logic has uploaded anything. After Logic uploads later, QGC does not automatically re-download even though ArduPilot broadcasts `MISSION_CURRENT(total=N)`.

**Fix:** `PushMissionToGCS` step in `simulator/planner/actions/upload_mission.py`. After upload to ArduPilot completes, Logic opens a UDP socket to `127.0.0.1:14550` (QGC port) and sends `MISSION_COUNT(N)`. QGC enters "downloading" state; its UDPLink broadcasts `MISSION_REQUEST_INT` to all known hosts including ArduPilot; ArduPilot serves items directly. QGC displays the mission.

Controlled by `push_to_gcs=True` (default) on `make_upload_mission()`.

> **Confidence:** Confirmed from current chat; fix implemented

---

## [FIXED] QGC "Mission transfer failed. Not accepting any mission commands." (~50% of runs)

**Symptom:** QGC shows "Mission transfer failed. Error: Not accepting any mission commands." approximately half the time at startup. After the error, mission never appears.

**Root cause (two-part):**

1. **Race condition:** `make_upload_mission` was the first action in `AutoPlan`, running at simulation start while QGC's `InitialConnectStateMachine` was still downloading parameters. QGC's `MISSION_REQUEST_LIST` arrived while ArduPilot had `receiving=true` (our upload in progress) → ArduPilot returned `MAV_MISSION_DENIED` → QGC never retried.

2. **`ClearMission` (from_scratch=True) triggered re-download:** `MISSION_CLEAR_ALL` caused ArduPilot to broadcast `MISSION_CURRENT(total=0)`, prompting QGC to send `MISSION_REQUEST_LIST` again → same race.

**Fixes applied:**

1. Moved `make_upload_mission` to **after** `Plan.arm()` in `AutoPlan.__init__()`. `Plan.arm()` / `make_pre_arm()` takes 10–30 s, long enough for QGC to finish its initial download before upload begins.
2. Changed `from_scratch=False` (default) to skip `ClearMission`.
3. Added DENIED backoff in `SendMissionCount.check_fn()`: if ArduPilot returns DENIED, wait 1 s and retry `MISSION_COUNT`.
4. Added `_MissionCache` to reduce upload duration (1 file read instead of N).

> **Confidence:** Confirmed from current chat; fix implemented and verified

---

## [FIXED] `CheckEndMission` Fires Immediately (HEARTBEAT Pollution)

**Symptom:** In `2-single_sim_plane.ipynb`, the mission ends immediately (same second as LAND item) instead of waiting ~48 s for the plane to actually land.

**Root cause:** `logic.py` sends a heartbeat to ArduPilot via the same TCP connection (`ap_conn`). ArduPilot routes this heartbeat back to Logic. `MAVLinkManager.run()` stores it in `state["HEARTBEAT"]` with `base_mode=0` (DISARMED), making `CheckEndMission` see the vehicle as disarmed while it is still flying.

**Fixes applied:**

1. `simulator/runtime/vehicle/mav_manager.py`: skip HEARTBEAT if `srcComponent != 1` (only accept autopilot heartbeats, `MAV_COMP_ID_AUTOPILOT1`).
2. `simulator/planner/actions/monitoring.py` `CheckEndMission`: added `_was_armed` flag; only fires on ARMED→DISARMED transition to prevent stale pre-arm heartbeats from triggering early.

> **Confidence:** Confirmed from previous session (memory: `project_arduplaneplane_sim.md`)

---

## [KNOWN] QGC "PreArm: In landing sequence" Popup After Mission Completion

**Symptom:** After the simulation finishes and the vehicle disarms, QGC shows a red popup: "PreArm: In landing sequence".

**Root cause:** QGC automatically sends `MAV_CMD_RUN_PREARM_CHECKS` approximately 12 seconds after it detects vehicle disarm. ArduPilot still considers itself in a landing sequence at that point and responds with `STATUSTEXT "PreArm: In landing sequence"`. QGC displays this as a warning popup.

**Impact:** Cosmetic only — the mission plan is already complete before this fires. Does not affect simulation results.

**Status:** No QGC ini key found to suppress this behavior. Attempted fix via `SYSID_MYGCS=1` + `MAV7_OPTIONS=1` in `vehicle.parm` **broke mission upload** (see next entry) and was reverted.

**Potential future fix:** Intercept and drop `PreArm:` STATUSTEXT messages in the Logic proxy after the plan reaches `DONE` state, preventing them from reaching QGC.

> **Confidence:** Root cause confirmed; fix pending

---

## [DO NOT USE] `SYSID_MYGCS=1` + `MAV7_OPTIONS=1` in vehicle.parm

**Symptom:** Adding these parameters caused "Mission transfer failed. Error: Not accepting any mission commands." in QGC.

**Root cause:** `MAV7_OPTIONS` likely restricts which sysid/component can send mission commands to ArduPilot, breaking the Logic process's mission upload. `SYSID_MYGCS=1` may also conflict with Logic's GCS identity.

**Action:** Do not add `SYSID_MYGCS` or `MAVx_OPTIONS` to `vehicle.parm`. Revert immediately if added. Delete `simulator/ardupilot_logs/veh_N/eeprom.bin` after reverting to clear cached params.

> **Confidence:** Confirmed; changes reverted

---

## [FIXED] `Model.__call__` Case Mismatch — Gazebo Always Gets Wrong Model Path

**Symptom:** `FileNotFoundError: .../ardupilot_gazebo/models/zephyr/ardupilot` — the raw
StrEnum value (`"zephyr"`) is used instead of the resolved Gazebo name (`"gazebo-zephyr"`).

**Root cause (two-part):**
1. `Model.__call__` compared `visualizer_name == "gazebo"` (lowercase) but `Gazebo.name`
   returns `"Gazebo"` (capital G) → comparison always False → falls through to non-Gazebo branch.
2. `Gazebo.get_visvehicle` passed `vehicle.model` (the raw `Model` enum) directly into
   `GazVehicle(model=...)` instead of calling `vehicle.model(self.name)` to resolve the string.
   But since `GazVehicle` should keep `model: Model`, the resolution must happen inside
   `_generate_vehicle_models_from_bases` via `model_name = veh.model(self.name)`.

**Fixes:**
- `config.py` `Model.__call__`: `visualizer_name.lower() == "gazebo"`.
- `gazebo.py` `_generate_vehicle_models_from_bases`: `model_name = veh.model(self.name)` at
  the top of each loop; use `model_name` for all path building and SDF string replacement.

> **Confidence:** Confirmed; fix implemented.

---

## [FIXED] SITL Crashes on `port_offset=0` — "EOF on TCP socket" / "bind failed"

**Symptom:** `create_tcp_conn` gets repeated `EOF on TCP socket` when `port_offset=0` (SITL base_port=5760). High offsets (e.g. 320) work fine.

**Root cause (two-part):**

1. **SITL binds multiple sequential TCP ports, not just the base.** With `--base-port 5760`:
   - SERIAL0 = 5760, SERIAL1 = 5762, SERIAL2 = 5763, SERIAL5 = 5765, …
   - Port 5765 = `BasePort.RID_DOWN`. An Oracle ZMQ `PUB` socket from a previous simulation run in **another Jupyter kernel** holds port 5765. SITL fails: `bind failed on port 5765 - Address already in use` and exits code 1.
   - SITL briefly owns port 5760 (detectable by `wait_for_port`), then crashes. Pymavlink connects, gets EOF as SITL dies.

2. **`is_port_open` was killing SITL.** The original helper used a raw socket connect to probe if the port was open. ArduPilot SITL exits when a client connects and immediately drops the connection. This was inadvertently killing SITL during startup polling.

**Fixes applied:**

- `simulator/helpers/connections/ports.py`:
  - Replaced `is_port_open` (socket-connect probe) with `is_port_listening` using `ss -tlnH` — checks LISTEN state without connecting.
  - Added `startup_delay=1.0` to `wait_for_port` — gives SITL time to finish binding all serial ports before the first MAVLink connection attempt.

- `simulator/helpers/cleanup.py` `_kill_stale_sim_sockets`:
  - Replaced `ss --kill` (requires root, always fails) with `fuser -k -KILL port/tcp` — kills any same-user process holding ports in the simulation range [5760, 5780). Works without root.

- `simulator/oracle.py`:
  - Added `_active: set[Oracle]` module-level registry and `close()` method.
  - `clean()` calls `_close_oracles()` first to release ZMQ sockets held by the **current** kernel before `fuser` handles other kernels.

> **Confidence:** Root cause confirmed by SITL log (`bind failed on port 5765`); fix tested end-to-end.

---

## [FIXED] GCS Intervention Never Triggered — `recv_match` Drain Bug

**Symptom:** `GCS._monitor_vehicle` never fired the intervention even though MISSION_CURRENT
messages were flowing from SITL. No "GCS intervention" log line appeared.

**Root cause (two-part):**

1. **Message drain:** `_is_vehicle_plan_done` called `conn.recv_match(type="STATUSTEXT", blocking=False)`. pymavlink's `recv_match` loops internally, reading and **discarding** all non-matching messages until it either finds a match or exhausts the socket buffer. Every loop iteration consumed and dropped all pending `MISSION_CURRENT` messages before the intervention check ever ran.

2. **Missing SITL→GCS telemetry forwarding:** Even after fixing (1), the GCS could never see `MISSION_CURRENT` because `MAVLinkManager` read all SITL messages into its internal state but never forwarded them to `cs_conn` (the UDP channel to the GCS). The GCS only received heartbeats and `LOGIC_DONE` from Logic.

**Fixes applied:**

1. `simulator/gcs.py` `_monitor_vehicle`: replaced the two competing `recv_match` calls with a **single blocking `recv_match` loop** that dispatches on message type — one branch handles `STATUSTEXT/LOGIC_DONE`, another handles `MISSION_CURRENT` for the intervention trigger. `_is_vehicle_plan_done` was deleted.

2. `simulator/runtime/vehicle/mav_manager.py` `MAVLinkManager`: added optional `gcs_conn: MAVConnection | None` parameter. In `run()`, messages whose type is in `_GCS_TELEMETRY_TYPES` are forwarded to `gcs_conn` after updating internal state. `logic.py` passes `cs_conn` as `gcs_conn`.

> **Confidence:** Confirmed from GCS log showing "GCS intervention" line appearing after both fixes applied.

---

## [KNOWN] `mypy` / `ruff` Not in Primary `.venv`

`ruff` and `mypy` are not installed in the main `.venv` (only in `.venv_broken`). Use `uv run python -c "..."` for quick import checks. For linting, use the tools via their full path in `.venv_broken/bin/` if needed, or run `uv sync` to reinstall dev dependencies.

> **Confidence:** Confirmed from current chat (tool not found errors)

---

## [KNOWN] `mission_type` Removed from pymavlink Dialect

**Symptom:** `TypeError` about unexpected `mission_type` argument when constructing `MAVLink_mission_item_int_message`.

**Root cause:** The `ardupilot` submodule was updated; the regenerated pymavlink dialect no longer includes `mission_type` as a constructor argument.

**Fix:** Remove `mission_type` from `mission_item_to_int()` call in `upload_mission.py`. Constructor takes 13 args, not 14.

> **Confidence:** Confirmed from previous session
