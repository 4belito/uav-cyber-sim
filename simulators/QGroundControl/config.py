"""QGorundContorl configuration class."""

from dataclasses import dataclass

from helpers.change_coordinates import ENUs_to_GRAs
from mavlink.customtypes.location import ENUPoses, GRAPose


@dataclass
class ConfigQGC:
    """Handles QGroundControl configuration based on UAV offsets and origin."""

    def __init__(self, offsets: ENUPoses, origin: GRAPose):
        self.origin = origin
        self.spawns = ENUs_to_GRAs(origin, offsets)

    def __repr__(self):
        return f"Origin={self.origin}, Spawns={self.spawns}"
