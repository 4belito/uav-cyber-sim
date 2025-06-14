"""
QGroundControl (QGC) visualizer module for UAV-CYBER-SIM.

This module defines the QGC class, a Simulator subclass that automates the launch of
QGroundControl and configures it to connect to multiple ArduPilot UAV instances via TCP.
It modifies the QGroundControl.ini file to set up connection links for each UAV.
"""

import os
import subprocess
from dataclasses import dataclass
from typing import List

from config import QGC_INI_PATH, QGC_PATH, BasePort
from helpers.change_coordinates import Offset, find_spawns
from params.simulation import CONNECT_GCS_TO_ARP
from plan import Plan
from simulators.sim import Simulator, VisualizerName


@dataclass
class ConfigQGroundControl:
    """Handles QGroundControl configuration based on UAV offsets and origin."""

    def __init__(self, offsets: List[Offset], origin: Offset):
        self.origin = origin
        self.spawns = find_spawns(origin, offsets)

    def __repr__(self):
        return f"Origin={self.origin}, Spawns={self.spawns}"


class QGC(Simulator):
    """
    QGroundControl visualizer class.

    This class manages the launch and setup of QGroundControl as the visual interface
    for monitoring and interacting with multiple UAVs. It automatically updates the
    QGroundControl.ini file to add or remove TCP link configurations.

    """

    def __init__(
        self,
        offsets: List[Offset],
        plans: List[Plan],
        origin: Offset,
        visible_terminals: bool = False,
    ):
        super().__init__(
            name=VisualizerName.QGROUND,
            offsets=offsets,
            plans=plans,
            visible_terminals=visible_terminals,
            delay_visualizer=True,
        )
        self.config: ConfigQGroundControl = ConfigQGroundControl(offsets, origin)

    def _add_vehicle_cmd_fn(self, i: int):
        spawn_str = ",".join(map(str, self.config.spawns[i]))
        return f" --custom-location={spawn_str}"

    def _launch_visualizer(self):
        self._delete_all_links()  # delete TCP
        if CONNECT_GCS_TO_ARP:
            # self._disable_autoconnect_udp()
            self._add_tcp_links(n=self.n_uavs)
        sim_cmd = [os.path.expanduser(QGC_PATH), "--appimage-extract-and-run"]
        # pylint: disable=consider-using-with
        subprocess.Popen(
            sim_cmd,
            stdout=subprocess.DEVNULL,  # Suppress standard output
            stderr=subprocess.DEVNULL,  # Suppress error output
            shell=False,  # Ensure safety when passing arguments
        )
        print(
            "🗺️ QGroundControl launched for 2D visualization — simulation powered "
            "by ArduPilot SITL."
        )

    def _delete_all_links(self):
        with open(QGC_INI_PATH, "r", encoding="utf-8") as f:
            lines = f.readlines()

        inside_links = False
        new_lines: List[str] = []

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

    def _disable_autoconnect_udp(self):
        """
        Disables QGroundControl's automatic UDP connection (usually on port 14550)
        by updating the [AutoConnect] section in the QGroundControl.ini file.
        """
        with open(QGC_INI_PATH, "r", encoding="utf-8") as file:
            lines = file.readlines()

        new_lines: List[str] = []
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

    def _add_tcp_links(self, n: int = 1, step: int = 10):
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
        new_lines: List[str] = []
        for i in range(n):
            idx = count + i
            port = BasePort.QGC + self.port_offsets[i]
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
        lines[count_line_idx + len(new_lines)] = f"count={count + n}\n"

        with open(QGC_INI_PATH, "w", encoding="utf-8") as file:
            file.writelines(lines)
