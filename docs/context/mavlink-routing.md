# MAVLink Routing & Port Architecture

## Port Map (from `simulator/config.py`)

Ports are split into two enums so the two kinds never interleave.

### `VehPort` — per-vehicle (`base + the vehicle's offset`)

| Constant | Value | Description |
|---|---|---|
| `VehPort.ARP` | `5760 + offset` | ArduPilot SITL TCP port (Logic ↔ SITL) |
| `VehPort.ADSB` | `5761 + offset` | ADSB injector (Oracle → injector) |
| `VehPort.ARP2` | `5762 + offset` | ArduPilot SERIAL1 (TCP, auto-opened by SITL) |
| `VehPort.ARP3` | `5763 + offset` | ArduPilot SERIAL2 (TCP, auto-opened by SITL) |
| `VehPort.RID_UP` | `5764 + offset` | Remote ID Logic → Oracle |
| `VehPort.RID_DOWN` | `5765 + offset` | Remote ID Oracle → Logic |
| `VehPort.GCS_CMD` | `5766 + offset` | Command channel: any GCS → Logic (UDP) |
| `VehPort.MITM_TELEM` | `5767 + offset` | MITM telemetry listener — only when MITM enabled |
| `VehPort.MITM_CMD` | `5768 + offset` | MITM command listener — only when MITM enabled |
| *(spare)* | `5769 + offset` | Unused slot in the vehicle's block |

> Every `VehPort` now sits inside the vehicle's own one-stride-wide block
> (`5760 + offset` .. `5769 + offset`), MITM included, so one vehicle's ports can
> never reach another's however many vehicles run. The MITM ports are reserved
> with the block but only bound when a MITM is enabled.

### The block is shared with ArduPilot

SITL listens on `base_port + N` for every serial it leaves as TCP
(`_serial_path[]`, `AP_HAL_SITL/SITL_State.h`), so the vehicle's block is not
ours alone. The TCP/UDP split below is **load-bearing**: our TCP users sit only
on slots SITL leaves free, and the slots SITL does use we take as UDP — a UDP
bind and a TCP listen on the same number do not conflict.

```text
slot | ours                  | SITL serial (TCP)
 +0  | ARP           tcp     | SERIAL0
 +1  | ADSB/zmq      tcp     | -
 +2  | ARP2          tcp     | SERIAL1
 +3  | ARP3          tcp     | SERIAL2
 +4  | RID_UP/zmq    tcp     | -
 +5  | RID_DOWN/zmq  tcp     | SERIAL5 -> freed by our --serial5=uart:
 +6  | GCS_CMD       udp     | SERIAL6
 +7  | MITM_TELEM    udp     | SERIAL7
 +8  | MITM_CMD      udp     | SERIAL8
 +9  | (spare)               | -
```

Moving a ZMQ port onto a TCP serial slot, or +6/+7/+8 to a TCP transport, would
collide with SITL. `_ports_available` therefore probes **both** transports.

`SitlPort` holds the ports SITL claims per vehicle entirely on its own — `RCIN`
5501, `FG_VIEW` 5503, `SIM_OUT`/`SIM_IN` 9002/9003 (Gazebo), `IRLOCK` 9005, all
UDP. `-I N` adds `10*N` to each while it is at its default
(`SITL_cmdline.cpp:401-421`) and we override none, so they stride exactly like
`VehPort` and are reserved with the block even though nothing of ours binds them.

> **Why `SITL_INSTANCE_STRIDE` must be 10.** ArduPilot hardcodes it: `--instance|-I N`
> *"adds 10\*instance to all port numbers"*. `Simulator` derives the SITL
> instance as `port_offset // SITL_INSTANCE_STRIDE`, so any other stride desyncs a vehicle
> from its own SITL. `--base-port` suppresses the instance offset for the serial
> block (`if (_base_port == BASE_PORT)`), which is why the two do not double up.

### `GCS_TELEM_WINDOW` — a *window* per vehicle

| Constant | Value | Description |
|---|---|---|
| `GCS_TELEM_WINDOW` | `20000 + k + offset` | Telemetry: Logic → the vehicle's *k*-th GCS (UDP) |

Not a `VehPort`: every member there is **one** port per vehicle, whereas a
vehicle takes several ports here — one per GCS monitoring it.

### `SimPort` — one per simulation

| Constant | Value | Description |
|---|---|---|
| `SimPort.QGC` | `14550` | QGroundControl UDP port — **fixed**, set by QGC itself |
| `SimPort.ORC_DONE` | `14560` | ZMQ ROUTER/DEALER for the Oracle done signal |

> Offsets are multiples of `SITL_INSTANCE_STRIDE` (10), so a vehicle's window is
> exactly wide enough before the next vehicle's begins — which is why
> `SITL_INSTANCE_STRIDE` = **10 GCSs** is the ceiling on how many may monitor one vehicle
> (`_assign_telem_ports` raises past it). The window is claimed atomically with
> the rest of the vehicle's block and every port is derivable from its offset,
> so there is no separate pool and no privileged "first" listener.

> **The two groups exclude each other, in both directions.** A port search picks
> by probing but never *holds* what it picks (the probe socket closes at once)
> and nothing has launched yet, so probing alone cannot see the other group.
> `launch()` therefore claims the `SimPort`s **first** — there are only two and
> QGC's is immovable — then passes them as `reserved` to the vehicle search,
> which rejects any offset whose block would touch one. Without this a vehicle's
> `MITM_TELEM` reaches QGC's 14550 at ~556 vehicles and silently steals it.

> **Confidence:** Confirmed from repo (`simulator/config.py`, `simulator/sim.py`)

## Full Communication Diagram

```
                     TCP 5760+offset
ArduPilot SITL  ←────────────────────→  Logic
      │                                    │
      │  (telemetry forwarded by Logic)    │  UDP 20000+k+offset (telemetry)
      │                                    ├──────────────────────────────→ GCS
      │                                    │
      │                                    │  UDP 5766+offset  (commands)
      │                                    ←────────────────────────────── GCS
      │
      │  UDP 14550 (udpclient)
      └──────────────────────────────────→ QGroundControl
```

### Telemetry path (SITL → GCS)

`MAVLinkManager` reads every message from SITL on `ap_conn` (TCP 5760). Messages whose
type is in `_GCS_TELEMETRY_TYPES` are forwarded to `cs_conn` (`udpout:20000+k`), which the GCS
receives. Forwarded types: `HEARTBEAT`, `GLOBAL_POSITION_INT`, `MISSION_CURRENT`,
`STATUSTEXT`, `VFR_HUD`, `ATTITUDE`, `SYS_STATUS`.

### Command path (GCS → SITL)

The GCS creates a UDP sender on `VehPort.GCS_CMD` (5766 + offset). Logic creates a
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

## Vehicles with Zero, One or Many GCSs

A vehicle may be monitored by **any number of GCSs, including none**
(`SimVehicle.gcss` / `SimGCS.vehicles`, linked through `SimGCS.add_vehicle` or
`SimVehicle.assign_gcs`). Two consequences fall out of that, both handled by
`Simulator.launch()`:

### Process ownership

The GCS process is what spawns a vehicle's OS processes (SITL, socat, ADS-B
injector, Logic, MITM), so exactly one component must own each vehicle:

- **First GCS in `veh.gcss` owns it.** Its `VehicleConfig["launch"]` is `True`;
  it launches the processes and terminates them when the vehicle finishes.
- **Every other GCS gets `launch: False`** and only attaches to the running
  vehicle. Its `VehicleRuntime.processes` is empty, so the teardown path is a
  natural no-op.
- **A vehicle with no GCS is launched by the `Simulator` itself**
  (`_launch_unassigned_vehicles`), keeping its handles in
  `Simulator.unassigned_procs`.

Both paths call the same `simulator/runtime/vehicle_launcher.py::launch_vehicle`.

### One telemetry listener per (vehicle, GCS)

A UDP port has a single binder, so the vehicle's GCSs cannot share one:

- The vehicle's *k*-th GCS takes `GCS_TELEM_WINDOW + k + offset`
  (`Simulator._assign_telem_ports`), capped at `Simulator.max_gcss_per_veh`.
  The list is aligned index-by-index with `veh.gcss` and reaches each GCS as
  `VehicleConfig["telem_port"]`.
- Logic receives the whole list as `LogicConfig["gcs_telem_ports"]` and fans the
  telemetry out to all of them (`MAVLinkManager.gcs_conns`). With a MITM
  interposed Logic still writes one stream to `MITM_TELEM`, and the **MITM** does
  the fan-out (`--telem-ports`, one `telem-ack` backflow relay per GCS).

The **command** direction needs no fan-out: every GCS sends to the one
`GCS_CMD + veh_offset` (or `MITM_CMD`) receiver.

### Mission completion

Logic blocks on a `COMMAND_ACK` for its `LOGIC_DONE` `STATUSTEXT`, so
`send_done_msgs` walks **every** telemetry link — a vehicle watched by several
GCSs only finishes once they have all seen it. With **no** GCS the step is
skipped entirely; the vehicle still reports `DONE` to the Oracle over ZMQ from
`RIDManager.stop()` (identity `log-<sysid>`), so Oracle shutdown is unaffected.

> **Confidence:** Confirmed from repo (`simulator/sim.py`, `simulator/gcs.py`,
> `simulator/logic.py`, `simulator/mitm.py`)

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
picks these up on 5766 and forwards them to ArduPilot.

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
  `VehicleConfig`. The MITM process is launched by the vehicle's owner in
  `simulator/runtime/vehicle_launcher.py::launch_vehicle` (first, so its listeners
  are bound before Logic/GCS start sending) and tracked in
  `processes[SimProcess.MITM]`, so existing cleanup terminates it.
- **Only the two senders are retargeted** — Logic's telemetry sender → `MITM_TELEM`,
  GCS's command sender → `MITM_CMD`. The receivers (telemetry `20000+k`, `GCS_CMD` 5766)
  are unchanged, so the endpoints are otherwise unaware of the interposition.

```
telemetry:  Logic --MITM_TELEM(5767)--> [MITM] --telem(20000+k)--> GCS
commands:   GCS   --MITM_CMD(5768)-->   [MITM] --GCS_CMD(5766)--> Logic
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
