"""RIDData class definition."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, TypedDict

if TYPE_CHECKING:
    from simulator.helpers.coordinates import ENU, GRA


class RIDDict(TypedDict):
    """Serializable representation of RIDData for storage and transmission."""

    sysid: int
    gra_pos: dict[str, float]
    enu_pos: dict[str, float]
    enu_vel: dict[str, float]
    speed: float
    cog: float
    ele: float
    rel_alt: float
    hdg: float
    last_update: float


@dataclass
class RIDData:
    """Data class for Remote ID information."""

    sysid: int
    gra_pos: GRA  # global position
    enu_pos: ENU  # m/s relative to origin
    enu_vel: ENU  # m/s relative to uav
    speed: float  # m/s
    cog: float  # angle of course over ground,(0° = North, 90° = East)
    ele: float  # elevation angle,(0° = North, 90° = Up)
    rel_alt: float  # meters relative to takeoff
    hdg: float  # degrees - like cog but for uav heading
    last_update: float  # optional, handy for freshness checks

    def to_dict(self) -> RIDDict:
        """Convert RIDData into a serializable dictionary."""
        return {
            "sysid": self.sysid,
            "gra_pos": self.gra_pos._asdict(),
            "enu_pos": self.enu_pos._asdict(),
            "enu_vel": self.enu_vel._asdict(),
            "speed": self.speed,
            "cog": self.cog,
            "ele": self.ele,
            "rel_alt": self.rel_alt,
            "hdg": self.hdg,
            "last_update": self.last_update,
        }
