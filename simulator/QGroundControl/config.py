"""QGorundContorl configuration class."""

from dataclasses import dataclass

import folium

from config import Color
from helpers.coordinates import (
    GRA,
    ENUPose,
    ENUPoses,
    ENUs,
    GRAPose,
    GRAPoses,
    GRAs,
)
from simulator.visualizer import ConfigVis


@dataclass
class QGCWP:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    name: str
    pos: GRA
    color: Color = Color.GREEN


QGCTraj = list[QGCWP]


@dataclass
class Mission:
    """Represents a vehicle with a model and a trajectory."""

    traj: QGCTraj
    delay: int  # in sec
    land: bool = True
    speed: float = 5.0  # m/s


@dataclass
class QGCVehicle:
    """Represents a vehicle with a model and a trajectory."""

    home: GRAPose
    mission: Mission


QGCVehicles = list[QGCVehicle]
Missions = list[Mission]


class ConfigQGC(ConfigVis[QGCVehicle]):
    """
    Creates a trajectory from an (N, 3) array of waypoints as WaypointMarker
    objects.
    """

    def __init__(
        self,
        origin: GRAPose,
    ) -> None:
        super().__init__()
        self.origin = origin

    def add(
        self,
        base_path: ENUs | ENUPoses,
        base_home: ENUPose,
        color: Color = Color.BLUE,
        mission_delay: int = 0,  # sec
        land: bool = True,
        speed: float = 5.0,  # m/s
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home_path = base_home.to_abs_all(base_path)
        path = self.origin.to_abs_all(home_path)
        home = self.origin.to_abs(base_home)
        traj = ConfigQGC.create_mtraj(traj=path, color=color)
        mission = Mission(traj=traj, delay=mission_delay, land=land, speed=speed)
        self.add_vehicle(QGCVehicle(home=home, mission=mission))

    def __str__(self) -> str:
        lines = [
            "ConfigGazebo:",
            f"  origin: {self.origin}",
            f"  vehicles ({len(self.vehicles)}):",
        ]
        for v in self.vehicles:
            lines.append(f"      trajectory ({len(v.mission.traj)} waypoints):")
            for wp in v.mission.traj:
                lines.append(f"        {wp}")
        return "\n".join(lines)

    @staticmethod
    def create_mtraj(
        traj: GRAs | GRAPoses,
        color: Color = Color.GREEN,
    ) -> QGCTraj:
        """
        Create a trajectory from an (N, 3) array of waypoints as
        WaypointMarker objects.
        """
        mtraj: QGCTraj = []
        for i, pos in enumerate(traj):
            mtraj.append(QGCWP(name=f"wp_{i}", pos=GRA(*pos[:3]), color=color))
        return mtraj

    @staticmethod
    def show_trajs(mtrajs: list[QGCTraj]):
        """Display the vehicles trajectories and origin in GRA coordinates."""
        m = folium.Map(zoom_start=18)

        # Plot each traj
        for mtraj in mtrajs:  # add more colors if needed
            for i, wp in enumerate(mtraj):
                wp.pos.draw(m, f"pos_{i}", wp.color)

        return m
