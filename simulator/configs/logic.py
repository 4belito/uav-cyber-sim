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
