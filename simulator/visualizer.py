"""Visualizer module."""

from abc import ABC, abstractmethod
from typing import Generic

from helpers.coordinates import GRAPose

from .vehicle import SimVehicle, V


class Visualizer(ABC, Generic[V]):
    """Abstract base class for UAV simulation visualizers."""

    name: str
    delay = False

    def __init__(self, gra_origin: GRAPose) -> None:
        self.gra_origin = gra_origin
        self.vehicles: list[V] = []
        self.num_vehicles: int = 0

    @abstractmethod
    def launch(self, port_offsets: list[int]) -> None:
        """Launch the visualizer."""
        raise NotImplementedError

    @abstractmethod
    def get_vehicle(self, vehicle: SimVehicle) -> V:
        """Convert a Vehicle to the visualizer-specific vehicle type."""
        raise NotImplementedError

    @abstractmethod
    def show(self) -> None:
        """Show a stathic preview visualization."""
        raise NotImplementedError

    def add_vehicle_cmd(self, i: int) -> str:
        """Add optional command-line for the ith vehicle."""
        return ""

    def add_vehicle(self, vehicle: SimVehicle) -> None:
        """Add a vehicle to the visualizer."""
        veh = self.get_vehicle(vehicle)
        self.vehicles.append(veh)
        self.num_vehicles += 1

    def remove_vehicle_at(self, index: int) -> bool:
        """Remove a vehicle by index."""
        if 0 <= index < len(self.vehicles):
            del self.vehicles[index]
            self.num_vehicles -= 1
            return True
        return False

    def __str__(self):
        return self.name
