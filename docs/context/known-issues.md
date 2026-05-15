# Known Issues, Root Causes, and Fixes

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

## [KNOWN] `mypy` / `ruff` Not in Primary `.venv`

`ruff` and `mypy` are not installed in the main `.venv` (only in `.venv_broken`). Use `uv run python -c "..."` for quick import checks. For linting, use the tools via their full path in `.venv_broken/bin/` if needed, or run `uv sync` to reinstall dev dependencies.

> **Confidence:** Confirmed from current chat (tool not found errors)

---

## [KNOWN] `mission_type` Removed from pymavlink Dialect

**Symptom:** `TypeError` about unexpected `mission_type` argument when constructing `MAVLink_mission_item_int_message`.

**Root cause:** The `ardupilot` submodule was updated; the regenerated pymavlink dialect no longer includes `mission_type` as a constructor argument.

**Fix:** Remove `mission_type` from `mission_item_to_int()` call in `upload_mission.py`. Constructor takes 13 args, not 14.

> **Confidence:** Confirmed from previous session
