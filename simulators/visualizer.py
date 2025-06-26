"""Visualizer module."""

from abc import ABC, abstractmethod

from helpers.change_coordinates import ENUs_to_GRAs
from mavlink.customtypes.location import ENUPoses, GRAPose


class Visualizer(ABC):
    """Abstract base class for UAV simulation visualizers."""

    name: str

    def __init__(
        self,
        origin: GRAPose,
        enu_poses: ENUPoses,
    ):
        self.enu_poses = enu_poses
        self.poses = ENUs_to_GRAs(origin, enu_poses)
        self.n_uavs = len(enu_poses)

    def __str__(self):
        return self.name

    def add_vehicle_cmd(self) -> str:
        """Add optional command-line for vehicle i."""
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
