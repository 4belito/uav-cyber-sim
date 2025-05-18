"""
Simulation script that launches the full setup:
1. ArduPilot instances for each vehicle
2. Logic process for each vehicle
3. Optionally a simulator (None, QGroundControl, or Gazebo)
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
import platform
from subprocess import Popen
from typing import Any, List

from config import ARDUPILOT_VEHICLE_PATH, LOGS_PATH, PARAMS_PATH
from helpers.change_coordinates import Offset
from plan import Plan


class VisualizerName(str, Enum):
    """Simulator type options used to configure the simulation environment."""

    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"

    def __str__(self):
        return str(self.value)


import time


class Simulator:
    """
    Base simulator class to manage UAV vehicle processes and optional external
    simulators.

    Args:
        name (VisualizerName): Type of simulator to use.
        offsets (List[Offset]): Spawn offsets for each UAV.
        plans (List[Plan]): Mission plans for each UAV.
    """

    def __init__(
        self,
        name: VisualizerName = VisualizerName.NONE,
        offsets: List[Offset] | None = None,
        plans: List[Plan] | None = None,
    ):
        self.name = name
        self.config: Any | None = None
        self.offsets: List[Offset] = offsets or [(0, 0, 0, 0)]
        self.n_uavs: int = len(self.offsets)
        self.ardu_path: Path = ARDUPILOT_VEHICLE_PATH
        self.plans: List[Plan] = plans or [Plan.basic()]

    def launch(self) -> None:
        """Launches vehicle instances and the optional simulator."""
        self.launch_vehicles()
        self._launch_visualizer()

    def launch_vehicles(self) -> None:
        """Launches ArduPilot and logic processes for each UAV."""
        for i in range(self.n_uavs):
            sysid = i + 1
            veh_cmd = (
                f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {sysid} "
                f"--no-rebuild --use-dir={LOGS_PATH} --add-param-file {PARAMS_PATH} "
                f"--no-mavproxy "
            )
            # veh_cmd = (
            #     f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {sysid} "
            #     f"--no-rebuild --use-dir={LOGS_PATH} --add-param-file {PARAMS_PATH}"
            #     f"--no-add-14550-ports"
            # )
            veh_cmd += self._add_vehicle_cmd_fn(i)
            p = self.create_process(veh_cmd, after="exit", visible=True)
            print(f"🚀 Vehicle {sysid} launched (PID {p.pid})")

            time.sleep(0.6)  # Give some time to ardupilot to start
            logic_cmd = f"python3 proxy.py --sysid {sysid}"
            p = self.create_process(logic_cmd, after="exit", visible=True)
            print(f"🚀 Vehicle {sysid} logic launched (PID {p.pid})")

    def _add_vehicle_cmd_fn(self, _i: int) -> str:
        """Optional hook to additional command-line args."""
        return ""

    def _launch_visualizer(self) -> None:
        """Optional hook to launch a visual simulator or GUI application."""
        print("ℹ️  Running without visualization.")

    def create_process(
        self, cmd: str, after: str = "exit", visible: bool = True
    ) -> Popen[bytes]:
        """Launch a subprocess, optionally in a visible terminal."""
        if visible:
            if platform.system() == "Linux":
                return Popen(["gnome-terminal", "--", "bash", "-c", f"{cmd}; {after}"])
            raise OSError("Unsupported OS for visible terminal mode.")
        return Popen(cmd.split())

    def __repr__(self) -> str:
        return f"name: '{self.name}'\noffsets: {self.offsets}\nconfig: {self.config}"
