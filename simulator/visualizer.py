"""Visualizer module."""

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

V = TypeVar("V")  # Vehicle type


class ConfigVis(ABC, Generic[V]):
    """Base class for visualizer configurations that manage a list of vehicles."""

    def __init__(self) -> None:
        self.vehicles: list[V] = []
        self.n_vehicles: int = 0

    def add_vehicle(self, vehicle: V) -> None:
        """Add a vehicle to the configuration."""
        self.vehicles.append(vehicle)
        self.n_vehicles += 1

    def remove_vehicle_at(self, index: int) -> bool:
        """Remove a vehicle by index."""
        if 0 <= index < len(self.vehicles):
            del self.vehicles[index]
            self.n_vehicles -= 1
            return True
        return False


class Visualizer(ABC, Generic[V]):
    """Abstract base class for UAV simulation visualizers."""

    name: str

    def __init__(self, config: ConfigVis[V]) -> None:
        self.config = config

    def __str__(self):
        return self.name

    def add_vehicle_cmd(self, i: int) -> str:
        """Add optional command-line for the ith vehicle."""
        return ""

    @abstractmethod
    def launch(self, port_offsets: list[int], verbose: int = 1) -> None:
        """Launch the visualizer."""
        pass


class NoneVisualizer(Visualizer[int]):
    """No-op visualizer for headless simulation."""

    name = "none"

    def launch(self, port_offsets: list[int], verbose: int = 1) -> None:
        """Print a message indicating that no visualizer will be launched."""
        if verbose:
            print("🙈 Running without visualization.")
