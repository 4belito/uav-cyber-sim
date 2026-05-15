"""
QGroundControl (QGC) visualizer module for UAV-CYBER-SIM.

This module defines the QGC class, a Simulator subclass that automates the launch of
QGroundControl and configures it to connect to multiple ArduPilot UAV instances via TCP.
It modifies the QGroundControl.ini file to set up connection links for each UAV.
"""

import logging
import os
from dataclasses import dataclass

import folium
from IPython.display import display  # type: ignore

from simulator.config import QGC_PATH, BasePort, Color
from simulator.entities import SimVehicle, Vehicle
from simulator.helpers.coordinates import (
    GRA,
    GRAPose,
)
from simulator.helpers.processes import create_process
from simulator.visualizer.visualizer import Visualizer


@dataclass
class QGCMarker:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    name: str
    pos: GRA
    color: Color


QGCMarkers = list[QGCMarker]


@dataclass
class QGCVehicle(Vehicle):
    """Represents a vehicle with a model and a trajectory."""

    home: GRAPose
    marker_traj: QGCMarkers


class QGC(Visualizer[QGCVehicle]):
    """
    QGroundControl visualizer class.

    This class manages the launch and setup of QGroundControl as the visual interface
    for monitoring and interacting with multiple UAVs. It automatically updates the
    QGroundControl.ini file to add or remove TCP link configurations.

    """

    def __init__(
        self,
        gra_origin: GRAPose,
    ):
        super().__init__(gra_origin)
        self.markers: QGCMarkers = []

    @property
    def name(self) -> str:
        """Name of the visualizer."""
        return "QGroundControl"

    def add_sitl_args(self, vehicle: SimVehicle) -> list[str]:
        """Add QGroundControl telematry serial port."""
        return ["--serial6", f"udpclient:127.0.0.1:{BasePort.QGC}"]

    def home_str(self, vehicle: SimVehicle) -> str:
        """Add GRA location to the vehicle command."""
        visveh = self.vehicles[vehicle.sysid]
        return visveh.home.to_str()

    def launch(self, port_offsets: dict[int, int]):
        """Launch the Gazebo."""
        create_process(
            cmd=" ".join([os.path.expanduser(QGC_PATH), "--appimage-extract-and-run"]),
            visible=False,
            title="QGroundControl",
            suppress_output=True,
        )
        logging.info(
            "🗺️  QGroundControl launched for 2D visualization — simulation powered "
            "by ArduPilot SITL."
        )

    def show(self):
        """Display the vehicles trajectories and origin in GRA coordinates."""
        lat0, lon0, *_ = self.gra_origin
        m = folium.Map(location=[lat0, lon0], zoom_start=18)

        # Plot each UAV's path
        for marker in self.markers:  # add more colors if needed
            marker.pos.draw(m, marker.name, marker.color)
        display(m)

    def get_visvehicle(self, vehicle: SimVehicle):
        """Convert a SimVehicle to a QGCVehicle."""
        graunpose_origin = self.gra_origin.unpose()
        home = graunpose_origin.pose().to_abs(vehicle.home)
        mtraj: QGCMarkers = []
        for i, wp in enumerate(vehicle.waypoints):
            gra_wp = graunpose_origin.to_abs(wp)
            mtraj.append(
                QGCMarker(
                    name=f"veh{vehicle.sysid}_wp{i}",
                    pos=gra_wp,
                    color=vehicle.color,
                )
            )
        self.markers.extend(mtraj)
        return QGCVehicle(home=home, marker_traj=mtraj, model=vehicle.model)
