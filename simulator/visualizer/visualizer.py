"""Visualizer module."""

from abc import ABC, abstractmethod
from typing import Generic

from simulator.entities import SimVehicle, VehT
from simulator.helpers.coordinates import GRAPose


class Visualizer(ABC, Generic[VehT]):
    """Abstract base class for simulation visualizers."""

    def __init__(self, gra_origin: GRAPose) -> None:
        self.gra_origin = gra_origin
        self.vehicles: dict[int, VehT] = {}

    # ==================
    # Abstract interface
    # ==================

    @property
    @abstractmethod
    def name(self) -> str:
        """Name of the visualizer."""
        pass

    @abstractmethod
    def launch(self, port_offsets: list[int]) -> None:
        """Launch the visualizer."""
        pass

    @abstractmethod
    def get_visvehicle(self, vehicle: SimVehicle) -> VehT:
        """Convert a Vehicle to the visualizer-specific vehicle type."""
        pass

    @abstractmethod
    def show(self) -> None:
        """Show a stathic preview visualization."""
        pass

    # ===================================
    # Optional (subclasses may override)
    # ===================================

    def add_vehicle_cmd(self, vehicle: SimVehicle) -> str:
        """Add optional command-line for the ith vehicle."""
        return ""

    def add_sitl_args(self) -> str:
        """Add optional SITL arguments for a vehicle."""
        return ""

    # ==================
    # Base functionality
    # ==================

    def add_vehicle(self, vehicle: SimVehicle) -> None:
        """Add a vehicle to the visualizer."""
        visveh = self.get_visvehicle(vehicle)
        self.vehicles[vehicle.sysid] = visveh

    def remove_vehicle(self, sysid: int) -> bool:
        """Remove a vehicle by system ID."""
        if sysid in self.vehicles:
            del self.vehicles[sysid]
            return True
        return False

    @property
    def num_vehicles(self) -> int:
        """Return the number of vehicles in the visualizer."""
        return len(self.vehicles)

    def __str__(self):
        return self.name
