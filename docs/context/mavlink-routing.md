# MAVLink Routing & Port Architecture

## Port Map (from `simulator/config.py`)

| Constant | Value | Description |
|---|---|---|
| `BasePort.ARP` | `5760 + offset` | ArduPilot SITL TCP port (Logic → SITL) |
| `BasePort.ADSB` | `5761 + offset` | ADSB injector (Oracle → injector) |
| `BasePort.GCS` | `5766 + offset` | Logic UDP server for GCS telemetry monitor |
| `BasePort.QGC` | `14550` | QGroundControl UDP port — **fixed, not per-vehicle** |
| `BasePort.ORC_DONE` | `5767` | ZMQ ROUTER/DEALER for Oracle |
| `BasePort.RID_UP` | `5764 + offset` | Remote ID Logic → Oracle |
| `BasePort.RID_DOWN` | `5765 + offset` | Remote ID Oracle → Logic |

> **Confidence:** Confirmed from repo (`simulator/config.py`)

## Two Separate Connections to Each SITL Instance

```
ArduPilot SITL ←─TCP 5760+offset─→ Logic  ←─UDP 5766+offset─→ GCS monitor
ArduPilot SITL ←─UDP 14550 (udpclient)──→ QGroundControl
```

- **Logic ↔ SITL**: Logic is the TCP client. All mission upload/execution happens here.
- **QGC ↔ SITL**: ArduPilot is the **UDP client** (`--serial6 udpclient:127.0.0.1:14550`).
  QGC is the **UDP server** on port 14550. **All SITL instances share the same QGC port.**
- Logic and QGC are **independent links** — Logic cannot inject messages into the QGC link via ArduPilot.

> **Confidence:** Confirmed from repo (`simulator/visualizer/QGroundControl/qgc.py`, `simulator/config.py`)

## QGC InitialConnectStateMachine Timing

When QGC connects, it runs a state machine that:
1. Downloads parameters (~10–30 s depending on link latency)
2. **Then** downloads the mission via `MISSION_REQUEST_LIST`

QGC **never retries** a mission download if it receives `MAV_MISSION_DENIED`. This is hardcoded and not configurable via ini.

> **Confidence:** Confirmed from current chat (ArduPilot source + QGC binary analysis)

## ArduPilot `MissionItemProtocol` Behavior (Confirmed from Source)

File: `ardupilot/libraries/GCS_MAVLink/MissionItemProtocol.cpp`

| Scenario | ArduPilot behavior |
|---|---|
| `receiving=true` + get `MISSION_REQUEST_LIST` | Returns `MAV_MISSION_DENIED` |
| `receiving=true` + get `MISSION_REQUEST_INT` | Returns `MAV_MISSION_DENIED` |
| `receiving=false` + get `MISSION_REQUEST_INT` | **Serves the item directly via `get_item()`** — no "sending mode" check |
| `receiving=false` + get `MISSION_ACK` | Ignored |

Key insight: ArduPilot responds to `MISSION_REQUEST_INT` **even when not in "sending" mode**, as long as it is not currently receiving an upload (`receiving=false`).

> **Confidence:** Confirmed from repo (read `MissionItemProtocol.cpp:142-180`)

## `MISSION_CURRENT` and the `total` Field

ArduPilot populates `total` in `MISSION_CURRENT` broadcasts:
```cpp
// GCS_Common.cpp:677-697
mavlink_msg_mission_current_send(chan, seq, num_commands, mission_state, mission_mode);
```
`num_commands` = `mission.num_commands() - 1` (home excluded).

QGC may or may not auto-re-download when `total` changes depending on version. **Do not rely on this** — use `PushMissionToGCS` instead.

> **Confidence:** Confirmed from repo (`ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:677`)

## QGC UDPLink Multi-Host Behavior

QGC's `UDPLink` tracks all source addresses that send to port 14550. When QGC needs to respond (e.g., `MISSION_REQUEST_INT`), it sends to **all known hosts**. This means:
- If Logic opens a new UDP socket to QGC:14550 and sends `MISSION_COUNT`, QGC adds Logic's ephemeral port to its host list.
- Subsequent `MISSION_REQUEST_INT` from QGC goes to **both** ArduPilot's port and Logic's port.
- ArduPilot (not in `receiving` mode) will respond with `MISSION_ITEM_INT` directly from its mission store.

> **Confidence:** Confirmed from current chat (QGC source analysis + ArduPilot MissionItemProtocol confirmation)
