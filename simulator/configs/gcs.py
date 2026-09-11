"""TypedDict schemas for GCS runtime configuration."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, TypedDict

if TYPE_CHECKING:
    from simulator.helpers.processes import SimProcesses


class GCSVehicleConfig(TypedDict):
    """
    One vehicle's entry in a GCS process's config: how to bring it up and watch it.

    Built by `Simulator._build_veh_config` and written into the GCS config JSON.

    Identity and ports:

    - `sysid` — the vehicle's MAVLink system id.
    - `veh_port_offset` — this vehicle's slot in the port map; every `VehPort` it
      uses is `base + veh_port_offset`.
    - `telem_port` — UDP port *this* GCS listens on for the vehicle's telemetry;
      each GCS watching the same vehicle gets its own (one binder per port).
    - `launch` — whether this GCS owns the vehicle's OS processes (spawns and
      terminates them). Only the vehicle's first GCS does; the rest just monitor.

    Spawn commands (shell strings the owning GCS runs):

    - `ardupilot_cmd` — the ArduPilot SITL binary.
    - `logic_cmd` — `simulator.logic`, the per-vehicle MAVLink proxy / plan runner.
    - `socat_cmd` — the pty pair bridging SITL's serial to the ADS-B injector.
    - `adsb_cmd` — `simulator.adsb_injector`, injects nearby traffic.
    - `mitm_cmd` — `simulator.mitm` when a MITM is interposed, else `""`.

    Attacks:

    - `mitm` — whether a man-in-the-middle proxy sits on this vehicle's
      GCS<->Logic links (`mitm_cmd` is then non-empty).
    - `intervention` — a serialized `Intervention` (`{trigger, plan_spec, ...}`),
      or `None` for no GCS intervention.
    """

    sysid: int
    veh_port_offset: int
    telem_port: int
    launch: bool
    ardupilot_cmd: str
    logic_cmd: str
    socat_cmd: str
    adsb_cmd: str
    mitm: bool
    mitm_cmd: str
    intervention: dict[str, Any] | None


class GCSConfig(TypedDict):
    """
    A GCS process's full configuration.

    Mirrors the kwargs `simulator.gcs.GCS.__init__` takes — but this is just an
    organizational schema, not actually constructed or read by the runtime.

    Fields:

    - `name` — identifies this GCS (used in log filenames and its ZMQ identity).
    - `vehicles` — the `GCSVehicleConfig`s this GCS launches and/or monitors.
    - `oracle_port_offset` — port offset for this run's Oracle sockets.
    - `record_positions` — whether the GCS dumps the trajectories it observes to
      `data/trajectories_<name>.pkl`.
    - `terminals` — which per-vehicle subprocesses get their own visible terminal
      window; the rest run headless.
    - `suppress` — which subprocesses have their stdout/stderr discarded.
    """

    name: str
    vehicles: list[GCSVehicleConfig]
    oracle_port_offset: int
    record_positions: bool
    terminals: SimProcesses
    suppress: SimProcesses
