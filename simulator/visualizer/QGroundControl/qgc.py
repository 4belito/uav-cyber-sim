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

    def launch(self, port_offsets: list[int]):
        """Launch the Gazebo."""
        # self._delete_all_links()
        # self._enable_autoconnect_udp()
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

    # def _delete_all_links(self):
    #     with open(QGC_INI_PATH, "r", encoding="utf-8") as f:
    #         lines = f.readlines()

    #     inside_links = False
    #     new_lines: list[str] = []

    #     for line in lines:
    #         if line.strip() == "[LinkConfigurations]":
    #             inside_links = True
    #             new_lines.append(line)
    #             new_lines.append("count=0\n")  # reset count
    #             continue

    #         if inside_links:
    #             if line.startswith("Link") or line.startswith("count="):
    #                 continue  # skip all LinkX and count lines
    #             elif line.startswith("["):  # next section begins
    #                 inside_links = False

    #         new_lines.append(line)
    #     self._write_ini(new_lines)

    # def _write_ini(self, lines: list[str]):
    #     with open(QGC_INI_PATH, "w", encoding="utf-8") as f:
    #         f.writelines(lines)
    #         f.flush()
    #         os.fsync(f.fileno())

    # def _enable_autoconnect_udp(self):
    #     with open(QGC_INI_PATH, "r", encoding="utf-8") as f:
    #         lines = f.readlines()

    #     new_lines: list[str] = []
    #     in_autoconnect = False
    #     autoconnect_found = False
    #     udp_written = False

    #     for line in lines:
    #         stripped = line.strip()

    #         if stripped == "[AutoConnect]":
    #             in_autoconnect = True
    #             autoconnect_found = True
    #             new_lines.append(line)
    #             continue

    #         if in_autoconnect:
    #             if stripped.startswith("UDPLink="):
    #                 new_lines.append("UDPLink=true\n")
    #                 udp_written = True
    #                 continue
    #             elif stripped.startswith("[") and stripped != "[AutoConnect]":
    #                 in_autoconnect = False

    #         new_lines.append(line)

    #     if autoconnect_found and not udp_written:
    #         idx = next(
    #             i for i, line in enumerate(new_lines) if line.strip() == "[AutoConnect]"
    #         )
    #         new_lines.insert(idx + 1, "UDPLink=true\n")
    #     elif not autoconnect_found:
    #         new_lines.append("\n[AutoConnect]\nUDPLink=true\n")

    #     self._write_ini(new_lines)
