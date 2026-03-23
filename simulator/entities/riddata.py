"""RIDData class definition."""

from dataclasses import dataclass

from simulator.helpers.coordinates import ENU, GRA


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
