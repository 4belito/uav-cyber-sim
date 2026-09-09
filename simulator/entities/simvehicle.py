"""Simulator Vehicle definitions."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Self

from simulator.config import VEH_PARAMS_PATH, Color, Model
from simulator.entities.simgcs import SimGCS
from simulator.entities.vehicle import Vehicle
from simulator.helpers.coordinates import ENUPose, ENUs
from simulator.planner.plan import Plan

if TYPE_CHECKING:
    from simulator.runtime.mitm.strategies import MITMStrategy


@dataclass(kw_only=True)
class SimVehicle(Vehicle):
    """Simulator vehicle class."""

    model: Model
    sysid: int
    home: ENUPose
    color: Color
    plan: Plan
    waypoints: ENUs
    # A vehicle may be monitored by zero, one or several GCSs. The first entry
    # owns the vehicle's OS processes; see `SimGCS` for the other side.
    gcss: list[SimGCS] = field(default_factory=lambda: [])
    # SITL defaults appended to the firmware's own, per vehicle. Either one file
    # or several stacked in order (later files override earlier parameters), so a
    # small overlay like `no_avoidance.parm` can sit on top of the shared base.
    parm: str | Sequence[str] = str(VEH_PARAMS_PATH)
    port_offset: int | None = None
    # Man-in-the-middle interposed on this vehicle's GCS<->Logic links. `None`
    # (the default) means no MITM. See `simulator.runtime.mitm.strategies`.
    mitm: MITMStrategy | None = None

    def __post_init__(self) -> None:
        # Back-link any GCS passed straight to the constructor.
        for gcs in list(self.gcss):
            gcs.add_vehicle(self)

    @property
    def parms(self) -> list[str]:
        """SITL defaults for this vehicle, normalized to an ordered list."""
        return [self.parm] if isinstance(self.parm, str) else list(self.parm)

    def spoof_spec(self) -> dict[str, Any] | None:
        """
        Return the serialized RID spoof profile for the logic process.

        `None` for an honest vehicle; `RIDSpoofer` overrides this to lie.
        """
        return None

    def set_port_offset(self, offset: int):
        """Set the port offset for the vehicle."""
        self.port_offset = offset

    def assign_gcs(self, gcs: SimGCS) -> None:
        """
        Have `gcs` monitor this vehicle.

        Updates both sides of the relation and is idempotent.
        """
        gcs.add_vehicle(self)

    def unassign_gcs(self, gcs: SimGCS) -> None:
        """Stop having `gcs` monitor this vehicle, unlinking both sides."""
        gcs.remove_vehicle(self)

    @property
    def owner_gcs(self) -> SimGCS | None:
        """
        GCS responsible for launching this vehicle's processes.

        The first GCS assigned, or `None` when the vehicle is unmonitored (the
        Simulator then launches it directly).
        """
        return self.gcss[0] if self.gcss else None

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
        mitm: MITMStrategy | None = None,
        parm: str | Sequence[str] = str(VEH_PARAMS_PATH),
    ) -> Self:
        """Create a SimVehicle from poses given relative to an ENU origin."""
        enu_home = enu_origin.to_abs(relative_home)
        return cls(
            sysid=sysid,
            gcss=list(gcss),
            parm=parm,
            home=enu_home,
            color=color,
            plan=plan,
            waypoints=ENUPose.unpose_all(enu_home.to_abs_all(relative_path)),
            model=model,
            mitm=mitm,
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
