"""TypedDict schemas for logic runtime configuration."""

from typing import Any, TypedDict


class LogicConfig(TypedDict):
    """UAV logic configuration."""

    sysid: int
    gra_origin_dict: dict[str, float]
    port_offset: int
    plan_spec: dict[str, Any]
