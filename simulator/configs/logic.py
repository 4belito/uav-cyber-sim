"""TypedDict schemas for logic runtime configuration."""

from typing import Any, NotRequired, TypedDict


class LogicConfig(TypedDict):
    """UAV logic configuration."""

    sysid: int
    veh_port_offset: int
    oracle_port_offset: int
    gra_origin_dict: dict[str, float]
    plan_spec: dict[str, Any]
    home_heading: NotRequired[float]
    mitm: NotRequired[bool]
    # UDP ports of every GCS monitoring this vehicle. Empty when the vehicle is
    # unmonitored, in which case no telemetry is emitted and no GCS ack awaited.
    gcs_telem_ports: NotRequired[list[int]]
    # Remote ID broadcast rate in Hz, set on the Oracle.
    rid_frequency: NotRequired[int]
    # RID spoof profile (a serialized `SpoofProfile`) for a spoofing vehicle;
    # absent/None for an honest one. See `simulator.entities.spoofer`.
    spoof: NotRequired[dict[str, Any] | None]
