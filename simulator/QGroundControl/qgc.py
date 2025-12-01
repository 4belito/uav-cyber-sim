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

from config import QGC_INI_PATH, QGC_PATH, BasePort, Color
from helpers.coordinates import (
    GRA,
    GRAPose,
)
from helpers.processes import create_process
from simulator.vehicle import SimVehicle, Vehicle
from simulator.visualizer import Visualizer


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


QGCVehicles = list[QGCVehicle]


class QGC(Visualizer[QGCVehicle]):
    """
    QGroundControl visualizer class.

    This class manages the launch and setup of QGroundControl as the visual interface
    for monitoring and interacting with multiple UAVs. It automatically updates the
    QGroundControl.ini file to add or remove TCP link configurations.

    """

    name = "QGroundControl"
    delay = True

    def __init__(
        self,
        gra_origin: GRAPose,
    ):
        super().__init__(gra_origin)
        self.markers: QGCMarkers = []

    def add_vehicle_cmd(self, i: int):
        """Add GRA location to the vehicle command."""
        homes_str = self.vehicles[i].home.to_str()
        return f" --custom-location={homes_str}"

    def launch(self, port_offsets: list[int]):
        """Launch the Gazebo."""
        self._delete_all_links()
        self._add_tcp_links(port_offsets)
        sim_cmd = [os.path.expanduser(QGC_PATH), "--appimage-extract-and-run"]
        create_process(
            cmd=" ".join(sim_cmd),
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

    def get_vehicle(self, vehicle: SimVehicle):
        """Convert a Vehicle to a QGCVehicle with GRA home position and trajectory."""
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
        return QGCVehicle(home=home, marker_traj=mtraj)

    def _delete_all_links(self):
        with open(QGC_INI_PATH, "r", encoding="utf-8") as f:
            lines = f.readlines()

        inside_links = False
        new_lines: list[str] = []

        for line in lines:
            if line.strip() == "[LinkConfigurations]":
                inside_links = True
                new_lines.append(line)
                new_lines.append("count=0\n")  # reset count
                continue

            if inside_links:
                if line.startswith("Link") or line.startswith("count="):
                    continue  # skip all LinkX and count lines
                elif line.startswith("["):  # next section begins
                    inside_links = False

            new_lines.append(line)

        with open(QGC_INI_PATH, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
            f.flush()

    def _disable_autoconnect_udp(self):
        """
        Disables QGroundControl's automatic UDP connection (usually on port 14550)
        by updating the [AutoConnect] section in the QGroundControl.ini file.
        """
        with open(QGC_INI_PATH, "r", encoding="utf-8") as file:
            lines = file.readlines()

        new_lines: list[str] = []
        in_autoconnect = False
        autoconnect_found = False
        udp_written = False

        for line in lines:
            stripped = line.strip()

            if stripped == "[AutoConnect]":
                in_autoconnect = True
                autoconnect_found = True
                new_lines.append(line)
                continue

            if in_autoconnect:
                if stripped.startswith("UDPLink="):
                    new_lines.append("UDPLink=false\n")
                    udp_written = True
                    continue
                elif stripped.startswith("[") and stripped != "[AutoConnect]":
                    in_autoconnect = False

            new_lines.append(line)

        if autoconnect_found and not udp_written:
            # We're inside AutoConnect but no UDPLink was present
            idx = next(
                i for i, line in enumerate(new_lines) if line.strip() == "[AutoConnect]"
            )
            new_lines.insert(idx + 1, "UDPLink=false\n")
        elif not autoconnect_found:
            # Append new section
            new_lines.append("\n[AutoConnect]\nUDPLink=false\n")

        with open(QGC_INI_PATH, "w", encoding="utf-8") as file:
            file.writelines(new_lines)

    def _add_tcp_links(self, port_offsets: list[int]):
        with open(QGC_INI_PATH, "r", encoding="utf-8") as file:
            lines = file.readlines()

        section_header = "[LinkConfigurations]"
        start_idx = None
        count = 0

        # Find existing [LinkConfigurations] section
        for idx, line in enumerate(lines):
            if line.strip() == section_header:
                start_idx = idx
                break

        # If section doesn't exist, create it at the end
        if start_idx is None:
            lines.append(f"\n{section_header}\n")
            lines.append("count=0\n")
            start_idx = len(lines) - 2  # index of the new section header
            count_line_idx = start_idx + 1
        else:
            # Get current count if section exists
            try:
                count_line_idx = next(
                    i
                    for i in range(start_idx, len(lines))
                    if lines[i].startswith("count=")
                )
                count = int(lines[count_line_idx].split("=")[1])
            except StopIteration:
                # count= line was not found, create one
                count_line_idx = start_idx + 1
                lines.insert(count_line_idx, "count=0\n")
                count = 0

        # Prepare new link entries
        new_lines: list[str] = []
        n_ports = len(port_offsets)
        for i in range(n_ports):
            idx = count + i
            port = BasePort.QGC + port_offsets[i]
            new_lines.extend(
                [
                    f"Link{idx}\\auto=true\n",
                    f"Link{idx}\\high_latency=false\n",
                    f"Link{idx}\\host=127.0.0.1\n",
                    f"Link{idx}\\name=drone{idx + 1}\n",
                    f"Link{idx}\\port={port}\n",
                    f"Link{idx}\\type=2\n",
                ]
            )

        # Insert new lines just before count=
        lines[count_line_idx:count_line_idx] = new_lines
        lines[count_line_idx + len(new_lines)] = f"count={count + n_ports}\n"

        with open(QGC_INI_PATH, "w", encoding="utf-8") as file:
            file.writelines(lines)
