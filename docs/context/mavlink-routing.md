# MAVLink Routing & Port Architecture

## Port Map (from `simulator/config.py`)

| Constant | Value | Description |
|---|---|---|
| `BasePort.ARP` | `5760 + offset` | ArduPilot SITL TCP port (Logic ↔ SITL) |
| `BasePort.ADSB` | `5761 + offset` | ADSB injector (Oracle → injector) |
| `BasePort.ARP2` | `5762 + offset` | ArduPilot SERIAL1 (TCP, auto-opened by SITL) |
| `BasePort.ARP3` | `5763 + offset` | ArduPilot SERIAL2 (TCP, auto-opened by SITL) |
| `BasePort.RID_UP` | `5764 + offset` | Remote ID Logic → Oracle |
| `BasePort.RID_DOWN` | `5765 + offset` | Remote ID Oracle → Logic |
| `BasePort.GCS` | `5766 + offset` | Telemetry channel: Logic → GCS (UDP) |
| `BasePort.ORC_DONE` | `5767` | ZMQ ROUTER/DEALER for Oracle done signal |
| `BasePort.GCS_CMD` | `5768 + offset` | Command channel: GCS → Logic (UDP) |
| `BasePort.MITM_TELEM` | `9000 + offset` | MITM telemetry listener (Logic → MITM → GCS) — only when MITM enabled |
| `BasePort.MITM_CMD` | `9001 + offset` | MITM command listener (GCS → MITM → Logic) — only when MITM enabled |
| `BasePort.QGC` | `14550` | QGroundControl UDP port — **fixed, not per-vehicle** |

> MITM ports sit at 9000/9001 (well above the per-UAV cluster) so a vehicle's
> block never collides with another vehicle's MITM ports at +10 stride. They are
> always reserved by the offset search but only bound when a MITM is enabled.

> **Confidence:** Confirmed from repo (`simulator/config.py`)

## Full Communication Diagram

```
                     TCP 5760+offset
ArduPilot SITL  ←────────────────────→  Logic
      │                                    │
      │  (telemetry forwarded by Logic)    │  UDP 5766+offset  (telemetry)
      │                                    ├──────────────────────────────→ GCS
      │                                    │
      │                                    │  UDP 5768+offset  (commands)
      │                                    ←────────────────────────────── GCS
      │
      │  UDP 14550 (udpclient)
      └──────────────────────────────────→ QGroundControl
```

### Telemetry path (SITL → GCS)

`MAVLinkManager` reads every message from SITL on `ap_conn` (TCP 5760). Messages whose
type is in `_GCS_TELEMETRY_TYPES` are forwarded to `cs_conn` (`udpout:5766`), which the GCS
receives. Forwarded types: `HEARTBEAT`, `GLOBAL_POSITION_INT`, `MISSION_CURRENT`,
`STATUSTEXT`, `VFR_HUD`, `ATTITUDE`, `SYS_STATUS`.

### Command path (GCS → SITL)

The GCS creates a UDP sender on `BasePort.GCS_CMD` (5768 + offset). Logic creates a
matching UDP receiver and runs a `GCSCommandForwarder` thread
(`simulator/runtime/vehicle/gcs_cmd_forwarder.py`) that forwards whitelisted commands
from the GCS to ArduPilot on `ap_conn`. Whitelisted types: `MISSION_COUNT`,
`MISSION_ITEM_INT`, `MISSION_ITEM`, `MISSION_REQUEST_LIST`, `MISSION_CLEAR_ALL`,
`MISSION_ACK`, `COMMAND_LONG`, `COMMAND_INT`, `SET_MODE`.

> **Note:** `GCS_CMD` is **not** a heartbeat channel. Logic opens the receiver with
> `wait_hb=False` so it doesn't block waiting for a heartbeat that will never arrive.

### QGC path (SITL → QGC, direct)

ArduPilot is the **UDP client** (`--serial6 udpclient:127.0.0.1:14550`). QGC is the UDP
server on port 14550. **All SITL instances share the same QGC port.** Logic and QGC are
independent links — Logic cannot inject messages into the QGC link via ArduPilot.

> **Confidence:** Confirmed from repo (`simulator/visualizer/QGroundControl/qgc.py`, `simulator/config.py`)

## GCS Intervention API

`Simulator.intervention` accepts a dict that is serialized into the GCS config JSON:

```python
simulator.intervention = {
    "trigger_seq": 3,       # fire when MISSION_CURRENT.seq >= this value
    "target_lat": ...,      # DO_REPOSITION target (degrees)
    "target_lon": ...,
    "target_alt": ...,      # metres (relative)
}
```

`GCS._monitor_vehicle` polls MISSION_CURRENT in its single recv loop. When the trigger
fires once, `GCS._send_intervention` sends `SET_MODE(GUIDED, custom_mode=4)` +
`COMMAND_INT(MAV_CMD_DO_REPOSITION=192)` via `cmd_conn`. Logic's `GCSCommandForwarder`
picks these up on 5768 and forwards them to ArduPilot.

**MISSION_CURRENT seq mapping (ArduCopter AutoPlan, 3-waypoint example):**

| seq | Event |
|-----|-------|
| 0 | Home / sitting on ground |
| 1 | TAKEOFF active (climbing) |
| 2 | Flying to first nav WP |
| 3 | Flying to second nav WP ← typical intervention point |

> **Confidence:** Confirmed from `7-gcs_intervention_sanity.ipynb` run logs

## Man-in-the-Middle (MITM) Interposition

`simulator/mitm.py` runs a per-vehicle proxy that sits transparently between the
GCS and the vehicle's Logic. Enabled via `simulator.mitm = {"strategy": "..."}`
(default strategy `passthrough`). When set:

- Sim writes `mitm=True` into the logic config and `mitm` + `mitm_cmd` into each
  `VehicleConfig`. The MITM process is launched by the GCS in `_launch_vehicle`
  (first, so its listeners are bound before Logic/GCS start sending) and tracked
  in `processes[SimProcess.MITM]`, so existing cleanup terminates it.
- **Only the two senders are retargeted** — Logic's telemetry sender → `MITM_TELEM`,
  GCS's command sender → `MITM_CMD`. The receivers (`GCS` 5766, `GCS_CMD` 5768)
  are unchanged, so the endpoints are otherwise unaware of the interposition.

```
telemetry:  Logic --MITM_TELEM(9000)--> [MITM] --GCS(5766)--> GCS
commands:   GCS   --MITM_CMD(9001)-->   [MITM] --GCS_CMD(5768)--> Logic
```

**Each link is proxied bidirectionally.** The telemetry channel is not purely
downlink: the `LOGIC_DONE` → `COMMAND_ACK` handshake flows back over it. A naive
one-way forwarder would swallow the ACK and the sim would hang in
`send_msg_until_ack`. So the MITM runs four `_Relay` threads — `downlink` and
`uplink` carry the strategy hooks; `telem-ack` and `cmd-reply` are transparent
backflow. Messages are forwarded as raw bytes (`dst.write(msg.get_msgbuf())`)
for true transparency; a strategy that synthesises a message has it re-packed
with the relay's own encoder so a destination connection's parser is never
touched from two threads.

**Strategy seam:** `simulator/runtime/mitm/strategies.py` defines `MITMStrategy`
with `on_downlink(msg)` / `on_uplink(msg)` → return msg (forward), modified msg,
or `None` (drop), plus `bind(ctx)` which hands the strategy a `MITMContext` for
**originating** traffic (`inject_to_logic` / `inject_to_gcs`, packed with
srcSystem 255 to spoof the GCS). Strategies receive params via the `params`
field of `MITMConfig`, serialized into the proxy's `--params '<json>'` arg.
New attacks subclass and `register_strategy(name, cls)`; the notebook selects by
name and supplies params.

**Built-in strategies:**

| Name | Behavior |
|---|---|
| `passthrough` | Forward everything unmodified (default). |
| `blackout` | Drop all uplink commands and all downlink telemetry **except** `HEARTBEAT` (GCS blocks on `wait_heartbeat` at startup) and the `LOGIC_DONE` STATUSTEXT (GCS completion). Attacker keeps the link looking alive while blinding the operator. |
| `hijack` | Watch `MISSION_CURRENT` on the relayed downlink; once `seq >= trigger_seq`, inject `SET_MODE(GUIDED)` + `DO_REPOSITION` toward `target_lat/lon/alt` — the GCS intervention, but attacker-driven and GCS-spoofed. Telemetry still passes through (visible hijack). |

> **Why blackout must keep HEARTBEAT/LOGIC_DONE:** the GCS's `_launch_vehicle`
> calls `conn.wait_heartbeat()` (blocks at startup) and `_monitor_vehicle` only
> exits on `LOGIC_DONE`. Dropping either deadlocks the run.

> **Confidence:** Confirmed — passthrough, blackout, and hijack all verified
> end-to-end against the real proxy (downlink/uplink/backflow forwarding, blackout
> suppression with heartbeat+LOGIC_DONE passthrough, hijack injection of
> SET_MODE+DO_REPOSITION at the trigger seq). Notebooks `8-mitm_passthrough`,
> `9-mitm_blackout`, `10-mitm_hijack`.

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
