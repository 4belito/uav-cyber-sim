"""RID-spoofing vehicle: a `SimVehicle` that carries a `SpoofProfile`."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Self

from simulator.config import VEH_PARAMS_PATH, Color, Model
from simulator.entities.simgcs import SimGCS
from simulator.entities.simvehicle import SimVehicle
from simulator.entities.spoof_profile import SpoofProfile
from simulator.helpers.coordinates import ENUPose, ENUs
from simulator.planner.plan import Plan


@dataclass(kw_only=True)
class RIDSpoofer(SimVehicle):
    """
    A vehicle that flies a real mission but lies about its position over Remote ID.

    Identical to `SimVehicle` except for `spoof`, which the Simulator threads to
    this vehicle's `RIDManager`; `spoof=None` broadcasts honestly. Collision
    avoidance is unchanged here — control it through `parm` as for any vehicle.
    """

    spoof: SpoofProfile | None = None

    def spoof_spec(self) -> dict[str, Any] | None:
        """
        Return the serialized spoof profile for the logic process.

        `None` when this spoofer is broadcasting honestly.
        """
        return self.spoof.to_dict() if self.spoof is not None else None

    @classmethod
    def from_relative(
        cls,
        model: Model,
        sysid: int,
        color: Color,
        plan: Plan,
        enu_origin: ENUPose,
        relative_home: ENUPose,  # relative to enu_origin
        relative_path: ENUs,  # relative waypoints
        gcss: Sequence[SimGCS] = (),
        parm: str | Sequence[str] = str(VEH_PARAMS_PATH),
        spoof: SpoofProfile | None = None,
    ) -> Self:
        """Create a spoofer from poses given relative to an ENU origin."""
        veh = super().from_relative(
            model=model,
            sysid=sysid,
            color=color,
            plan=plan,
            enu_origin=enu_origin,
            relative_home=relative_home,
            relative_path=relative_path,
            gcss=gcss,
            parm=parm,
        )
        veh.spoof = spoof
        return veh
