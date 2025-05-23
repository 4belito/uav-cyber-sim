import os
import subprocess
from simulators.sim import Simulator, SimName
from helpers.change_coordinates import find_spawns
from plan import Plan
from typing import List, Tuple
from config import QGC_PATH, QGC_INI_PATH


class QGC(Simulator):
    def __init__(self, offsets: List[Tuple], plans: List[Plan], origin: Tuple):
        super().__init__(name=SimName.QGROUND, offsets=offsets, plans=plans)
        self.add_info("origin", origin)
        self.add_info("spawns", find_spawns(origin, offsets))

    def _add_vehicle_cmd_fn(self, i):
        spawn_str = ",".join(map(str, self.info["spawns"][i]))
        return f" --custom-location={spawn_str}"

    def _launch_application(self):
        # This is for connect manually using TCP
        delete_all_qgc_links()
        add_qgc_links(n=self.n_uavs)
        sim_cmd = [os.path.expanduser(QGC_PATH)]
        subprocess.Popen(
            sim_cmd,
            stdout=subprocess.DEVNULL,  # Suppress standard output
            stderr=subprocess.DEVNULL,  # Suppress error output
            shell=False,  # Ensure safety when passing arguments
        )

    # This is for TCP connections
    def launch(self, verbose: int = 1):
        super().launch_vehicles()
        uavs = super().create_VehicleLogics(verbose)
        self._launch_application()
        return uavs


def add_qgc_links(n: int = 1, start_port: int = 5763, step: int = 10):
    with open(QGC_INI_PATH, "r") as file:
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
                i for i in range(start_idx, len(lines)) if lines[i].startswith("count=")
            )
            count = int(lines[count_line_idx].split("=")[1])
        except StopIteration:
            # count= line was not found, create one
            count_line_idx = start_idx + 1
            lines.insert(count_line_idx, "count=0\n")
            count = 0

    # Prepare new link entries
    new_lines = []
    for i in range(n):
        idx = count + i
        port = start_port + step * i
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
        # print(f"✅ QGroundControl connected to port {port}")

    # Insert new lines just before count=
    lines[count_line_idx:count_line_idx] = new_lines
    lines[count_line_idx + len(new_lines)] = f"count={count + n}\n"

    with open(QGC_INI_PATH, "w") as file:
        file.writelines(lines)


def delete_all_qgc_links():
    with open(QGC_INI_PATH, "r") as f:
        lines = f.readlines()

    inside_links = False
    new_lines = []

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

    with open(QGC_INI_PATH, "w") as f:
        f.writelines(new_lines)
