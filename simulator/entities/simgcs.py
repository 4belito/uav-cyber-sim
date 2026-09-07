"""Simulator GCS configuration entity."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simulator.helpers.processes import SimProcess

if TYPE_CHECKING:
    from simulator.entities.simvehicle import SimVehicle


@dataclass
class SimGCS:
    """
    Simulator GCS configuration.

    A GCS monitors zero or more vehicles, and a vehicle may be monitored by
    zero, one or several GCSs. `SimGCS.vehicles` and `SimVehicle.gcss` are the
    two sides of that many-to-many relation; always link them through
    `add_vehicle` (or `SimVehicle.assign_gcs`) so both sides stay consistent.

    The first GCS a vehicle is assigned to owns the vehicle's OS processes
    (SITL, logic, ADS-B, MITM); the remaining ones only monitor it.

    `verbose`/`terminals`/`suppress` override the Simulator-wide defaults for
    this GCS only; leave them `None` to inherit the Simulator's settings.
    `record_positions` decides whether this GCS logs the trajectories it sees.
    """

    name: str
    # repr/compare are off to avoid recursing back through SimVehicle.gcss.
    vehicles: list[SimVehicle] = field(
        default_factory=lambda: [], repr=False, compare=False
    )
    # Record each vehicle's GLOBAL_POSITION_INT into `data/trajectories_<name>.pkl`,
    # read by `Oracle.plot_trajectories(gcss=True)`. Turn off to skip the file.
    record_positions: bool = False
    verbose: int | None = None
    terminals: list[SimProcess] | None = None
    suppress: list[SimProcess] | None = None

    def __post_init__(self) -> None:
        # Back-link any vehicle passed straight to the constructor, mirroring
        # `SimVehicle.__post_init__`; without this the vehicle would not know
        # about the GCS and the two sides of the relation would disagree.
        for vehicle in list(self.vehicles):
            self.add_vehicle(vehicle)

    @property
    def sysids(self) -> list[int]:
        """System IDs of the monitored vehicles, in assignment order."""
        return [veh.sysid for veh in self.vehicles]

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

    def remove_vehicle(self, vehicle: SimVehicle) -> None:
        """Stop monitoring `vehicle`, unlinking both sides of the relation."""
        self.vehicles = [veh for veh in self.vehicles if veh is not vehicle]
        vehicle.gcss = [gcs for gcs in vehicle.gcss if gcs is not self]
