"""TypedDict schemas for GCS runtime configuration."""

from typing import TypedDict

from simulator.helpers.processes import SimProcess


class VehicleConfig(TypedDict):
    """TypedDict for UAV configuration in the GCS."""

    sysid: int
    veh_port_offset: int
    ardupilot_cmd: str
    logic_cmd: str
    socat_cmd: str
    adsb_cmd: str


# Note: This is a simple configuration schema for the GCS.
# It is just for organization but is not being used
class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    vehicles: list[VehicleConfig]
    oracle_port_offset: int
    terminals: list[SimProcess]
    suppress: list[SimProcess]
