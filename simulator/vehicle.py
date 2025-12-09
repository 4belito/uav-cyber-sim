"""Vehicle definitions."""

from __future__ import annotations

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

    @classmethod
    def from_relative(
        cls,
        sysid: int,
        gcs_name: str,
        color: Color,
        plan: Plan,
        enu_origin: ENUPose,
        relative_home: ENUPose,  # relative to enu_origin
        relative_path: ENUs,  # relative waypoints
        model: str = "iris",
    ) -> SimVehicle:
        """Create a SimVehicle from poses given relative to an ENU origin."""
        enu_home = enu_origin.to_abs(relative_home)
        waypoints = enu_home.to_abs_all(relative_path)

        return cls(
            sysid=sysid,
            gcs_name=gcs_name,
            home=enu_home,
            color=color,
            plan=plan,
            waypoints=ENUPose.unpose_all(waypoints),
            model=model,
        )


SimVehicles = list[SimVehicle]
