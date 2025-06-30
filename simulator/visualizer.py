"""Visualizer module."""

from abc import ABC, abstractmethod


class Visualizer(ABC):
    """Abstract base class for UAV simulation visualizers."""

    name: str

    def __str__(self):
        return self.name

    def add_vehicle_cmd(self, i: int) -> str:
        """Add optional command-line for the ith vehicle."""
        return ""

    @abstractmethod
    def launch(self, port_offsets: list[int], verbose: int = 1) -> None:
        """Launch the visualizer."""
        pass


class NoneVisualizer(Visualizer):
    """No-op visualizer for headless simulation."""

    name = "none"

    def launch(self, port_offsets: list[int], verbose: int = 1) -> None:
        """Print a message indicating that no visualizer will be launched."""
        if verbose:
            print("🙈 Running without visualization.")
