"""Simulator GCS configuration entity."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simulator.entities.intervention import Intervention
    from simulator.entities.simvehicle import SimVehicle, SimVehicles
    from simulator.helpers.processes import SimProcesses


@dataclass
class SimGCS:
    """
    Simulator GCS configuration: which vehicles a ground station monitors and how.

    A GCS monitors zero or more vehicles, and a vehicle may be monitored by zero,
    one or several GCSs. `SimGCS.vehicles` and `SimVehicle.owner_gcs` /
    `SimVehicle.monitor_gcs` are the two sides of that many-to-many relation.
    `own` / `monitor` are the *only* way to establish it — there is no
    constructor argument on either class and no other method that does — so
    every link states its role outright; each raises `ValueError` if it would
    contradict an existing assignment instead of silently doing something
    else. `remove_vehicle` is the one way to undo either.

    `owned_vehicles` / `monitored_vehicles` split `vehicles` explicitly by
    role; they are computed, not stored — `SimVehicle.owner_gcs` is the single
    place ownership is ever recorded, so there is no second copy to fall out of
    sync and nothing to reconcile between the two sides.

    Fields:

    - `vehicles` — every vehicle this GCS watches, owned or not; set only via
      `own` / `monitor` / `remove_vehicle`, never at construction. `repr`/
      `compare` are off to avoid recursing back through `SimVehicle.owner_gcs`
      / `SimVehicle.monitor_gcs`.
    - `record_positions` — when set, log each vehicle's `GLOBAL_POSITION_INT` to
      `data/trajectories_<name>.pkl`, read by `Oracle.plot_trajectories(gcss=True)`.
    - `verbose` / `terminals` / `suppress` — override the Simulator-wide defaults
      for this GCS only; leave `None` to inherit them.
    - `interventions` — per-vehicle-sysid map of `Intervention`s this GCS applies;
      when a trigger fires the GCS takes over that vehicle with its guided plan.
      Set it via `intervene`.
    """

    name: str
    record_positions: bool = False
    verbose: int | None = None
    terminals: SimProcesses | None = None
    suppress: SimProcesses | None = None
    interventions: dict[int, Intervention] = field(
        default_factory=dict[int, "Intervention"], repr=False, compare=False
    )
    vehicles: SimVehicles = field(
        default_factory=lambda: [], init=False, repr=False, compare=False
    )

    def __repr__(self) -> str:
        """Show the monitored sysids and which of them this GCS owns."""
        return (
            f"SimGCS(name={self.name!r}, monitors={self.sysids}, "
            f"owns={self.owned_sysids}, record_positions={self.record_positions}"
        )

    @property
    def sysids(self) -> list[int]:
        """System IDs of the monitored vehicles, in assignment order."""
        return [veh.sysid for veh in self.vehicles]

    @property
    def owned_vehicles(self) -> SimVehicles:
        """
        The monitored vehicles this GCS owns.

        A GCS owns a vehicle when it is that vehicle's `owner_gcs`; it then
        launches that vehicle's OS processes.
        """
        return [veh for veh in self.vehicles if veh.owner_gcs is self]

    @property
    def monitored_vehicles(self) -> SimVehicles:
        """The monitored vehicles this GCS does *not* own — it only watches them."""
        return [veh for veh in self.vehicles if veh.owner_gcs is not self]

    @property
    def owned_sysids(self) -> list[int]:
        """System IDs of `owned_vehicles`."""
        return [veh.sysid for veh in self.owned_vehicles]

    @property
    def monitored_sysids(self) -> list[int]:
        """System IDs of `monitored_vehicles`."""
        return [veh.sysid for veh in self.monitored_vehicles]

    def own(self, vehicle: SimVehicle) -> None:
        """
        Become `vehicle`'s owner: launch and control its OS processes.

        Raises `ValueError` if a *different* GCS already owns it — ownership
        never changes hands silently. Idempotent if this GCS already owns it.
        Registers `vehicle` in `self.vehicles` too, if it wasn't already.
        """
        if not any(veh is vehicle for veh in self.vehicles):
            self.vehicles.append(vehicle)
        if vehicle.owner_gcs is not None and vehicle.owner_gcs is not self:
            raise ValueError(
                f"Vehicle {vehicle.sysid} is already owned by "
                f"{vehicle.owner_gcs.name!r}; cannot also make "
                f"{self.name!r} its owner."
            )
        vehicle.owner_gcs = self

    def monitor(self, vehicle: SimVehicle) -> None:
        """
        Watch `vehicle`'s telemetry without owning it.

        Raises `ValueError` if this GCS already owns `vehicle` — demote it
        with `remove_vehicle` first if that's really what you want, rather
        than silently downgrading ownership here. Idempotent if this GCS
        already monitors it. Registers `vehicle` in `self.vehicles` too, if
        it wasn't already.
        """
        if not any(veh is vehicle for veh in self.vehicles):
            self.vehicles.append(vehicle)
        if vehicle.owner_gcs is self:
            raise ValueError(
                f"{self.name!r} already owns vehicle {vehicle.sysid}; "
                "cannot also add it as monitor-only."
            )
        if self not in vehicle.monitor_gcs:
            vehicle.monitor_gcs.append(self)

    def remove_vehicle(self, vehicle: SimVehicle) -> None:
        """
        Stop monitoring `vehicle`, unlinking both sides of the relation.

        If this GCS was the owner, the first remaining `monitor_gcs` entry (if
        any) is promoted to `owner_gcs` — same as a plain list losing its head.
        """
        self.vehicles = [veh for veh in self.vehicles if veh is not vehicle]
        if vehicle.owner_gcs is self:
            vehicle.owner_gcs = (
                vehicle.monitor_gcs.pop(0) if vehicle.monitor_gcs else None
            )
        else:
            vehicle.monitor_gcs = [
                gcs for gcs in vehicle.monitor_gcs if gcs is not self
            ]

    def intervene(self, vehicle: SimVehicle, intervention: Intervention) -> None:
        """
        Have this GCS intervene on a vehicle it monitors.

        Raises `ValueError` if `vehicle` is not monitored here (`own` / `monitor`
        it first). Most interventions assume the target flies an `AutoPlan` — see
        `Intervention` for which parts depend on it. Interventions are per-GCS; a
        repeat call for the same vehicle replaces the previous one.
        """
        if not any(veh is vehicle for veh in self.vehicles):
            raise ValueError(
                f"GCS {self.name} does not monitor vehicle {vehicle.sysid}; "
                "own() or monitor() it before setting an intervention."
            )
        self.interventions[vehicle.sysid] = intervention
