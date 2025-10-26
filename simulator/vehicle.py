"""Vehicle definitions."""

from dataclasses import dataclass, field
from typing import TypeVar

from config import Color
from helpers.coordinates import ENUPose, ENUs
from planner.plan import Plan


@dataclass
class Vehicle:
    """Base vehicle class."""


V = TypeVar("V", bound=Vehicle)


@dataclass
class SimVehicle(Vehicle):
    """Simulator vehicle class."""

    sysid: int
    gcs_name: str
    home: ENUPose
    color: Color
    plan: Plan
    waypoints: ENUs
    model: str = field(default="iris")
    port_offset: int = field(init=False, default=-1)

    def set_port_offset(self, offset: int):
        """Set the port offset for the vehicle."""
        self.port_offset = offset


SimVehicles = list[SimVehicle]
