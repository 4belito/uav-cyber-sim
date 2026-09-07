"""TypedDict schemas for GCS runtime configuration."""

from typing import TypedDict

from simulator.helpers.processes import SimProcess


class VehicleConfig(TypedDict):
    """TypedDict for UAV configuration in the GCS."""

    sysid: int
    veh_port_offset: int
    # UDP port this GCS listens on for the vehicle's telemetry. Each GCS
    # monitoring the same vehicle gets its own port (a port has one binder).
    telem_port: int
    # Whether this GCS owns the vehicle's OS processes (launch + terminate).
    # Only the vehicle's first GCS does; the others just monitor.
    launch: bool
    ardupilot_cmd: str
    logic_cmd: str
    socat_cmd: str
    adsb_cmd: str
    mitm: bool
    mitm_cmd: str
    intervention: dict[str, float] | None


# Note: This is a simple configuration schema for the GCS.
# It is just for organization but is not being used
class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    vehicles: list[VehicleConfig]
    oracle_port_offset: int
    record_positions: bool
    terminals: list[SimProcess]
    suppress: list[SimProcess]
