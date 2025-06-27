"""Gazebo configuration class."""

from dataclasses import dataclass

from config import Color
from mavlink.customtypes.location import ENU, ENUPose, ENUPoses, ENUs

COLOR_MAP: dict[Color, str] = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
}


@dataclass
class WPMarker:
    """
    Defines a visual waypoint marker with position, color, size, and transparency
    in Gazebo.
    """

    pos: ENU
    color: Color = Color.GREEN
    radius: float = 0.2
    alpha: float = 0.05

    def __repr__(self) -> str:
        return (
            f"WaypointMarker(pos={self.pos}, color='{self.color}', "
            f"radius={self.radius}, alpha={self.alpha})"
        )


MarkerTraj = list[WPMarker]
MarkerTrajs = list[MarkerTraj]
Model = tuple[str, Color]
Models = list[Model]


@dataclass
class ConfigGazebo:
    """
    Creates a trajectory from an (N, 3) array of waypoints as WaypointMarker
    objects.
    """

    world_path: str
    models: Models
    marker_trajs: MarkerTrajs

    def __str__(self):
        return (
            f"world_path={self.world_path}, models={self.models}, "
            f"markers={self.marker_trajs}"
        )

    @staticmethod
    def create_markertraj(
        traj: ENUs | ENUPoses,
        color: Color = Color.GREEN,
        radius: float = 0.2,
        alpha: float = 0.05,
    ) -> MarkerTraj:
        """
        Create a trajectory from an (N, 3) array of waypoints as
        WaypointMarker objects.
        """
        markertraj: MarkerTraj = []
        for pos in traj:
            if isinstance(pos, ENUPose):
                x, y, z, _ = pos
                pos = ENU(x, y, z)
            markertraj.append(
                WPMarker(
                    pos=pos,
                    color=color,
                    radius=radius,
                    alpha=alpha,
                )
            )

        return markertraj
