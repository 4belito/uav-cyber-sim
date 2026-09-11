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
    one or several GCSs. `SimGCS.vehicles` and `SimVehicle.gcss` are the two sides
    of that many-to-many relation; always link them through `add_vehicle` (or
    `SimVehicle.assign_gcs`) so both sides stay consistent. The first GCS a
    vehicle is assigned to owns the vehicle's OS processes (SITL, logic, ADS-B,
    MITM); the rest only monitor it.

    Fields:

    - `vehicles` — the monitored vehicles; `repr`/`compare` are off to avoid
      recursing back through `SimVehicle.gcss`.
    - `record_positions` — when set, log each vehicle's `GLOBAL_POSITION_INT` to
      `data/trajectories_<name>.pkl`, read by `Oracle.plot_trajectories(gcss=True)`.
    - `verbose` / `terminals` / `suppress` — override the Simulator-wide defaults
      for this GCS only; leave `None` to inherit them.
    - `interventions` — per-vehicle-sysid map of `Intervention`s this GCS applies;
      when a trigger fires the GCS takes over that vehicle with its guided plan.
      Set it via `intervene`.
    """

    name: str
    vehicles: SimVehicles = field(default_factory=lambda: [], repr=False, compare=False)
    record_positions: bool = False
    verbose: int | None = None
    terminals: SimProcesses | None = None
    suppress: SimProcesses | None = None
    interventions: dict[int, Intervention] = field(
        default_factory=dict[int, "Intervention"], repr=False, compare=False
    )

    def __post_init__(self) -> None:
        """Back-link any vehicle passed straight to the constructor."""
        self.add_vehicles(list(self.vehicles))

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
    def owned_sysids(self) -> list[int]:
        """
        System IDs of the monitored vehicles this GCS owns.

        A GCS owns a vehicle when it is the first GCS assigned to it (see
        `SimVehicle.owner_gcs`); it then launches that vehicle's OS processes.
        """
        return [veh.sysid for veh in self.vehicles if veh.gcss and veh.gcss[0] is self]

    def add_vehicle(self, vehicle: SimVehicle) -> None:
        """
        Monitor `vehicle` from this GCS.

        Both sides of the relation are updated, and the call is idempotent so
        assigning the same pair twice never duplicates it.
        """
        if not any(veh is vehicle for veh in self.vehicles):
            self.vehicles.append(vehicle)
        if not any(gcs is self for gcs in vehicle.gcss):
            vehicle.gcss.append(self)

    def add_vehicles(self, vehicles: SimVehicles) -> None:
        """Monitor multiple vehicles from this GCS."""
        for vehicle in vehicles:
            self.add_vehicle(vehicle)

    def remove_vehicle(self, vehicle: SimVehicle) -> None:
        """Stop monitoring `vehicle`, unlinking both sides of the relation."""
        self.vehicles = [veh for veh in self.vehicles if veh is not vehicle]
        vehicle.gcss = [gcs for gcs in vehicle.gcss if gcs is not self]

    def intervene(self, vehicle: SimVehicle, intervention: Intervention) -> None:
        """
        Have this GCS intervene on a vehicle it monitors.

        Raises `ValueError` if `vehicle` is not monitored here (`add_vehicle` it
        first). Most interventions assume the target flies an `AutoPlan` — see
        `Intervention` for which parts depend on it. Interventions are per-GCS; a
        repeat call for the same vehicle replaces the previous one.
        """
        if not any(veh is vehicle for veh in self.vehicles):
            raise ValueError(
                f"GCS {self.name} does not monitor vehicle {vehicle.sysid}; "
                "add_vehicle() it before setting an intervention."
            )
        self.interventions[vehicle.sysid] = intervention
