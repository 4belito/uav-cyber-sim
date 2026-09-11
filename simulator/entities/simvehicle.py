"""`SimVehicle`: a vehicle's identity, mission, and per-vehicle runtime config."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Self

from simulator.config import NO_AVOID_PARAMS_PATH, VEH_PARAMS_PATH, Color, Model
from simulator.entities.vehicle import Vehicle
from simulator.helpers.coordinates import ENUPose, ENUs

if TYPE_CHECKING:
    from collections.abc import Sequence

    from simulator.entities.simgcs import SimGCS
    from simulator.entities.spoof_profile import SpoofProfile
    from simulator.planner.plan import Plan
    from simulator.runtime.mitm.strategies import MITMStrategy


@dataclass(kw_only=True)
class SimVehicle(Vehicle):
    """
    A vehicle definition for the simulator.

    Fields beyond the base `Vehicle`:

    - `gcss` — the GCSs monitoring this vehicle (zero, one, or several). The
      first entry owns the vehicle's OS processes; `SimGCS` holds the other side
      of the relation, and `__post_init__` / `assign_gcs` keep both sides linked.
    - `parm` — SITL default-parameter file(s) for this vehicle, appended to the
      firmware's own. `None` (the default) uses the shared `vehicle.parm` base;
      pass one path or an ordered list to set your own base (later files win).
    - `avoidance` — when `False`, `parms` appends the `no_avoidance.parm` overlay
      on top of `parm` so this vehicle ignores ADS-B traffic and never yields
      (e.g. a body-blocker or an attacker). The overlay only touches
      `AVD_ENABLE` / `AVD_F_ACTION`, so it composes with any custom `parm`.
    - `mitm` — a man-in-the-middle strategy interposed on this vehicle's
      GCS<->Logic links; `None` (the default) means no MITM. See
      `simulator.runtime.mitm.strategies`.
    - `spoof` — a Remote ID `SpoofProfile` for this vehicle to lie with; `None`
      (the default) broadcasts honestly. Threaded to this vehicle's
      `RIDManager` via `spoof_spec`.
    - `port_offset` — assigned by the Simulator; read via `port_offset_required`.
    """

    sysid: int
    home: ENUPose
    color: Color
    plan: Plan
    waypoints: ENUs
    gcss: list[SimGCS] = field(default_factory=lambda: [])
    parm: str | Sequence[str] | None = None
    avoidance: bool = True
    port_offset: int | None = None
    mitm: MITMStrategy | None = None
    spoof: SpoofProfile | None = None

    def __post_init__(self) -> None:
        """Back-link any GCS passed straight to the constructor."""
        for gcs in list(self.gcss):
            gcs.add_vehicle(self)

    @property
    def parms(self) -> list[str]:
        """
        SITL default-parameter files for this vehicle, as an ordered stack
        (later files win).

        `parm` is the base: `None` uses the shared `vehicle.parm`, or pass your
        own path / list. When `avoidance` is `False`, the `no_avoidance.parm`
        overlay is appended last — it only flips `AVD_ENABLE` / `AVD_F_ACTION`,
        so the rest of the stack is untouched; just leave those two out of your
        own file.
        """
        if self.parm is None:
            stack = [str(VEH_PARAMS_PATH)]
        elif isinstance(self.parm, str):
            stack = [self.parm]
        else:
            stack = list(self.parm)
        if not self.avoidance:
            stack.append(str(NO_AVOID_PARAMS_PATH))
        return stack

    def spoof_spec(self) -> dict[str, Any] | None:
        """
        Return the serialized RID spoof profile for the logic process.

        `None` when `spoof` is unset, i.e. this vehicle broadcasts honestly.
        """
        return self.spoof.to_dict() if self.spoof is not None else None

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
        relative_home: ENUPose,
        relative_path: ENUs,
        gcss: Sequence[SimGCS] = (),
        mitm: MITMStrategy | None = None,
        parm: str | Sequence[str] | None = None,
        avoidance: bool = True,
        spoof: SpoofProfile | None = None,
    ) -> Self:
        """
        Create a SimVehicle from poses given relative to an ENU origin.

        `relative_home` is relative to `enu_origin`; `relative_path` are
        waypoints relative to that home. Both are resolved to absolute ENU here.
        """
        enu_home = enu_origin.to_abs(relative_home)
        enu_waypoints = ENUPose.unpose_all(enu_home.to_abs_all(relative_path))
        return cls(
            sysid=sysid,
            gcss=list(gcss),
            parm=parm,
            avoidance=avoidance,
            home=enu_home,
            color=color,
            plan=plan,
            waypoints=enu_waypoints,
            model=model,
            mitm=mitm,
            spoof=spoof,
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
