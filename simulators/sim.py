"""
Simulation script that launches the full setup:
1. ArduPilot instances for each vehicle
2. Logic process for each vehicle
3. Optionally a simulator (None, QGroundControl, or Gazebo)
"""

from enum import Enum
import platform
import subprocess
from typing import Dict, List, Optional, Tuple

from config import ARDUPILOT_VEHICLE_PATH, LOGS_PATH, PARAMS_PATH
from plan import Plan
from vehicle_logic import VehicleLogic


class VisualizerName(str, Enum):
    """Simulator type options used to configure the simulation environment."""

    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"


Offset = Tuple[float, float, float, float]


class Simulator:
    """
    Base simulator class to manage UAV vehicle processes and optional external simulators.

    Args:
        name (VisualizerName): Type of simulator to use.
        offsets (List[Offset]): Spawn offsets for each UAV.
        plans (List[Plan]): Mission plans for each UAV.
    """

    def __init__(
        self,
        name: VisualizerName = VisualizerName.NONE,
        offsets: Optional[List[Offset]] = None,
        plans: Optional[List[Plan]] = None,
    ):
        self.name = name
        self.info: Dict[str, object] = {}
        self.offsets: List[Offset] = offsets or [(0, 0, 0, 0)]
        self.n_uavs: int = len(self.offsets)
        self.ardu_path: str = ARDUPILOT_VEHICLE_PATH
        self.plans: List[Plan] = plans or [Plan.basic()]

    def launch(self) -> List[VehicleLogic]:
        """Launches vehicle instances and the optional simulator."""
        self.launch_vehicles()
        self._launch_visualizer()

    def launch_vehicles(self) -> None:
        """Launches ArduPilot and logic processes for each UAV."""
        for i in range(self.n_uavs):
            sysid = i + 1
            veh_cmd = (
                f"python3 {self.ardu_path} -v ArduCopter -I{i} --sysid {sysid} "
                f"--no-rebuild --use-dir={LOGS_PATH} --add-param-file {PARAMS_PATH}"
            )
            veh_cmd += self._add_vehicle_cmd_fn(i)
            p = self.create_process(veh_cmd, after="exit", visible=True)
            print(f"🚀 Vehicle {sysid} launched (PID {p.pid})")

            logic_cmd = f"python3 proxy.py --sysid {sysid}"
            p = self.create_process(logic_cmd, after="exit", visible=True)
            print(f"🚀 Vehicle {sysid} logic launched (PID {p.pid})")

    def _add_vehicle_cmd_fn(self, i: int) -> str:
        """Hook to be overridden by subclasses for additional command-line args."""
        raise NotImplementedError("This method should be implemented in a subclass")

    def _launch_visualizer(self) -> None:
        """Optional hook to launch a visual simulator or GUI application."""
        print("ℹ️  Running without a simulator.")

    def add_info(self, key: str, value: object) -> None:
        """Store arbitrary metadata about the simulation run."""
        self.info[key] = value

    def create_process(
        self, cmd: str, after: str = "exit", visible: bool = True
    ) -> subprocess.Popen:
        """
        Launch a subprocess, optionally in a visible terminal.

        Args:
            cmd (str): The shell command to execute.
            after (str): Command to run after `cmd` (e.g., "exit" keeps terminal open).
            visible (bool): Whether to launch in a visible terminal (Linux only).

        Returns:
            subprocess.Popen: The created process handle.
        """
        if visible:
            if platform.system() == "Linux":
                return subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", f"{cmd}; {after}"]
                )
            raise OSError("Unsupported OS for visible terminal mode.")
        return subprocess.Popen(cmd.split())

    def __repr__(self) -> str:
        return f"name='{self.name}'\noffsets={self.offsets}\ninfo={self.info}"
