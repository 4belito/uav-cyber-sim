"""Simulator Vehicle definitions."""

from __future__ import annotations

from dataclasses import dataclass

from simulator.config import Color
from simulator.entities.vehicle import Vehicle
from simulator.helpers.coordinates import ENUPose, ENUs
from simulator.planner.plan import Plan


@dataclass(kw_only=True)
class SimVehicle(Vehicle):
    """Simulator vehicle class."""

    sysid: int
    gcs_name: str
    home: ENUPose
    color: Color
    plan: Plan
    waypoints: ENUs
    port_offset: int | None = None
    instance: int | None = None
    model: str = "+"
    firmware: str = "ArduCopter"

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
        model: str = "+",  # ("frame defines the model internally")
        firmware: str = "ArduCopter",
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
            firmware=firmware,
        )

    @property
    def port_offset_required(self) -> int:
        """Return the port offset for the vehicle, or raise an error if not set."""
        if self.port_offset is None:
            raise RuntimeError(
                f"Vehicle {self.sysid} port_offset has not been assigned"
            )
        return self.port_offset


SimVehicles = list[SimVehicle]
