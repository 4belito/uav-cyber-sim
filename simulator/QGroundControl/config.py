"""QGorundContorl configuration class."""

from dataclasses import dataclass

import folium
from IPython.display import display  # type: ignore

from config import Color
from helpers.change_coordinates import draw_grapose, pose, poses
from mavlink.customtypes.location import (
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

    pos: GRA
    color: Color = Color.GREEN


QGCTraj = list[QGCWP]


@dataclass
class Mission:
    """Represents a vehicle with a model and a trajectory."""

    traj: QGCTraj
    delay: int  # in sec
    n_items: int


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
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home_path = poses(base_home, base_path)
        path = poses(self.origin, home_path)
        home = pose(self.origin, base_home)
        traj = ConfigQGC.create_mtraj(traj=path, color=color)
        n_items = len(traj) + 2 + int(mission_delay > 0)
        mission = Mission(traj=traj, delay=mission_delay, n_items=n_items)
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

    def show(self, origin_color: Color = Color.WHITE):
        """Display the vehicles trajectories and origin in GRA coordinates."""
        lat0, lon0, *_ = self.origin
        m = folium.Map(location=[lat0, lon0], zoom_start=18)

        # Plot each UAV's path
        for veh in self.vehicles:  # add more colors if needed
            for i, wp in enumerate(veh.mission.traj):
                draw_grapose(m, wp.pos, f"pos_{i}", wp.color)

        # Plot origin
        draw_grapose(m, self.origin, "Origin", origin_color)
        display(m)
        return m

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
        for pos in traj:
            mtraj.append(QGCWP(pos=GRA(*pos[:3]), color=color))
        return mtraj

    @staticmethod
    def show_trajs(mtrajs: list[QGCTraj]):
        """Display the vehicles trajectories and origin in GRA coordinates."""
        m = folium.Map(zoom_start=18)

        # Plot each traj
        for mtraj in mtrajs:  # add more colors if needed
            for i, wp in enumerate(mtraj):
                draw_grapose(m, wp.pos, f"pos_{i}", wp.color)

        return m
