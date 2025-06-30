"""QGorundContorl configuration class."""

from dataclasses import dataclass

import folium

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
from mavlink.util import save_mission


@dataclass
class QGCWP:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    pos: GRA
    color: Color = Color.GREEN


QGCTraj = list[QGCWP]


@dataclass
class QGCVehicle:
    """Represents a vehicle with a model and a trajectory."""

    home: GRAPose
    mtraj: QGCTraj


QGCVehicles = list[QGCVehicle]


class ConfigQGC:
    """
    Creates a trajectory from an (N, 3) array of waypoints as WaypointMarker
    objects.
    """

    def __init__(
        self,
        origin: GRAPose,
    ) -> None:
        self.origin = origin
        self.vehicles: QGCVehicles = []

    def add_vehicle(
        self,
        mtraj: QGCTraj,
        home: GRAPose = GRAPose(0, 0, 0, 0),
    ) -> None:
        """Add a vehicle to the QGC configuration."""
        self.vehicles.append(QGCVehicle(home=home, mtraj=mtraj))

    def add(
        self,
        base_path: ENUs | ENUPoses,
        base_home: ENUPose,
        color: Color = Color.BLUE,
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home_path = poses(base_home, base_path)
        path = poses(self.origin, home_path)
        home = pose(self.origin, base_home)
        mtraj = ConfigQGC.create_mtraj(traj=path, color=color)
        self.add_vehicle(mtraj=mtraj, home=home)

    def remove_vehicle_at(self, index: int) -> bool:
        """Remove a vehicle by index."""
        if 0 <= index < len(self.vehicles):
            del self.vehicles[index]
            return True
        return False

    def __str__(self) -> str:
        lines = [
            "ConfigGazebo:",
            f"  origin: {self.origin}",
            f"  vehicles ({len(self.vehicles)}):",
        ]
        for v in self.vehicles:
            lines.append(f"      trajectory ({len(v.mtraj)} waypoints):")
            for wp in v.mtraj:
                lines.append(f"        {wp}")
        return "\n".join(lines)

    def show(self, origin_color: Color = Color.WHITE):
        """Display the vehicles trajectories and origin in GRA coordinates."""
        lat0, lon0, *_ = self.origin
        m = folium.Map(location=[lat0, lon0], zoom_start=18)

        # Plot each UAV's path
        for veh in self.vehicles:  # add more colors if needed
            for i, wp in enumerate(veh.mtraj):
                draw_grapose(m, wp.pos, f"pos_{i}", wp.color)

        # Plot origin
        draw_grapose(m, self.origin, "Origin", origin_color)
        return m

    def save_missions(self, missions_name: str):
        """Save the missions for all the vehicles."""
        for i, veh in enumerate(self.vehicles):
            traj = [wp.pos for wp in veh.mtraj]
            save_mission(name=f"{missions_name}_{i + 1}", poses=traj)

    @staticmethod
    def create_mtraj(
        traj: GRAs | GRAPoses,
        color: Color = Color.GREEN,
    ) -> QGCTraj:
        """
        Create a trajectory from an (N, 3) array of waypoints as
        WaypointMarker objects.
        """
        markertraj: QGCTraj = []
        for pos in traj:
            markertraj.append(QGCWP(pos=GRA(*pos[:3]), color=color))
        return markertraj
