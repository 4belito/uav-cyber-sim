"""Gazebo configuration class."""

from dataclasses import dataclass

from config import Color
from helpers.coordinates import (
    ENU,
    ENUPose,
    ENUPoses,
    ENUs,
)
from simulator.visualizer import ConfigVis

COLOR_MAP: dict[Color, str] = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
    Color.WHITE: "1.0 1.0 1.0 1",
}


@dataclass
class GazWP:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    name: str
    group: str
    pos: ENU
    color: Color = Color.GREEN
    radius: float = 0.2
    alpha: float = 0.05


GazTraj = list[GazWP]


@dataclass
class GazVehicle:
    """Represents a vehicle with a model and a trajectory."""

    model: str
    color: Color
    home: ENUPose
    mtraj: GazTraj


GazVehicles = list[GazVehicle]


class ConfigGazebo(ConfigVis[GazVehicle]):
    """Gazebo configuration and marker visualization manager."""

    def __init__(
        self,
        origin: ENUPose,
        world_path: str,
    ) -> None:
        super().__init__()
        self.origin = origin
        self.world_path = world_path
        self.traj_id = 0

    def add(
        self,
        base_path: ENUs | ENUPoses,
        base_home: ENUPose,
        color: Color = Color.BLUE,
        model: str = "iris",
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home_path = base_home.to_abs_all(base_path)
        path = self.origin.to_abs_all(home_path)
        home = self.origin.to_abs(base_home)
        mtraj = ConfigGazebo.create_mtraj(
            name=f"traj_{self.traj_id}", traj=path, color=color
        )
        self.add_vehicle(
            GazVehicle(
                model=model,
                color=color,
                home=home,
                mtraj=mtraj,
            )
        )
        self.traj_id += 1

    @staticmethod
    def create_mtraj(
        name: str,
        traj: ENUs | ENUPoses,
        color: Color = Color.GREEN,
        radius: float = 0.2,
        alpha: float = 0.05,
    ) -> GazTraj:
        """Create a trajectory from waypoints as GazWPMarker objects."""
        markertraj: GazTraj = []
        for i, pos in enumerate(traj):
            markertraj.append(
                GazWP(
                    name=str(i),
                    group=name,
                    pos=ENU(*pos[:3]),
                    color=color,
                    radius=radius,
                    alpha=alpha,
                )
            )
        return markertraj
