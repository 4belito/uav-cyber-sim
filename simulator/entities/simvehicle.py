"""Simulator Vehicle definitions."""

from __future__ import annotations

from dataclasses import dataclass

from simulator.config import Color, Model
from simulator.entities.simgcs import SimGCS
from simulator.entities.vehicle import Vehicle
from simulator.helpers.coordinates import ENUPose, ENUs
from simulator.planner.plan import Plan


@dataclass(kw_only=True)
class SimVehicle(Vehicle):
    """Simulator vehicle class."""

    model: Model
    sysid: int
    gcs: SimGCS
    home: ENUPose
    color: Color
    plan: Plan
    waypoints: ENUs
    port_offset: int | None = None
    instance: int | None = None

    def set_port_offset(self, offset: int):
        """Set the port offset for the vehicle."""
        self.port_offset = offset

    @classmethod
    def from_relative(
        cls,
        model: Model,
        sysid: int,
        gcs: SimGCS,
        color: Color,
        plan: Plan,
        enu_origin: ENUPose,
        relative_home: ENUPose,  # relative to enu_origin
        relative_path: ENUs,  # relative waypoints
    ) -> SimVehicle:
        """Create a SimVehicle from poses given relative to an ENU origin."""
        enu_home = enu_origin.to_abs(relative_home)
        return cls(
            sysid=sysid,
            gcs=gcs,
            home=enu_home,
            color=color,
            plan=plan,
            waypoints=ENUPose.unpose_all(enu_home.to_abs_all(relative_path)),
            model=model,
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
