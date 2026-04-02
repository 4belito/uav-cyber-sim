"""TypedDict schemas for GCS runtime configuration."""

from typing import TypedDict

from simulator.helpers.processes import SimProcess


class VehicleConfig(TypedDict):
    """TypedDict for UAV configuration in the GCS."""

    sysid: int
    port_offset: int
    ardupilot_cmd: str
    logic_cmd: str
    socat_cmd: str
    adsb_cmd: str


class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    port_offset: int
    uavs: list[VehicleConfig]
    terminals: list[SimProcess]
    suppress: list[SimProcess]
