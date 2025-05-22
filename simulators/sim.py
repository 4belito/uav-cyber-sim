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


from typing import Any
from pymavlink.mavutil import mavlink_connection as connect  # type: ignore
from config import ARDUPILOT_VEHICLE_PATH, LOGS_PATH, VEH_PARAMS_PATH
from helpers.change_coordinates import Offset
from helpers.mavlink import MAVConnection
from plan import Plan
from config import BasePort
from oracle import Oracle, GCS


class VisualizerName(str, Enum):
    """Simulator type options used to configure the simulation environment."""

    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"

    def __str__(self):
        return str(self.value)


class Simulator:
    """
    Base simulator class to manage UAV vehicle processes and optional external
    simulators.

    Args:
        name (VisualizerName): Type of simulator to use.
        offsets (list[Offset]): Spawn offsets for each UAV.
        plans (list[Plan]): Mission plans for each UAV.
    """

    def __init__(
        self,
        name: VisualizerName = VisualizerName.NONE,
        offsets: list[Offset] | None = None,
        plans: list[Plan] | None = None,
    ):
        self.name = name
        self.config: Any | None = None
        self.offsets: list[Offset] = offsets or [(0, 0, 0, 0)]
        self.n_uavs: int = len(self.offsets)
        self.ardu_path: Path = ARDUPILOT_VEHICLE_PATH
        self.plans: list[Plan] = plans or [Plan.basic()]

    def launch(self, gcs_sysids: dict[str, list[int]]) -> tuple[Oracle, list[GCS]]:
        """Launches vehicle instances and the optional simulator."""
        self._launch_visualizer()
        oracle, gcss = self.launch_vehicles(gcs_sysids)
        return oracle, gcss

    def launch_vehicles(
        self, gcs_sysids: dict[str, list[int]], oracle_name: str = "Oracle ⚪"
    ) -> tuple[Oracle, list[GCS]]:
        """Launches ArduPilot and logic processes for each UAV."""
        orc_conns: list[MAVConnection] = []
        gcs_conns: dict[str, list[MAVConnection]] = {name: [] for name in gcs_sysids}
        gcs_of = GCS.map_sysid_to_gcs(gcs_sysids)
        for i in range(self.n_uavs):
            sysid = i + 1
            veh_cmd = (
                f"python3 {self.ardu_path}"
                f" -v ArduCopter -I{i} --sysid {sysid} --no-rebuild"
                f" --use-dir={LOGS_PATH} --add-param-file {VEH_PARAMS_PATH}"
                f" --no-mavproxy"
            )
            veh_cmd += self._add_vehicle_cmd_fn(i)
            p = self.create_process(veh_cmd, after="exit", visible=True)
            print(f"🚀 Vehicle {sysid} launched (PID {p.pid})")

            logic_cmd = f"python3 proxy.py --sysid {sysid}"
            p = self.create_process(
                logic_cmd, after="exit", visible=True
            )  # "exec bash"
            print(f"🚀 Vehicle {sysid} logic launched (PID {p.pid})")

            ## Connect to oracle
            port = BasePort.ORC + 10 * (sysid - 1)
            conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
            conn.wait_heartbeat()
            print(f"🔗 UAV logic {sysid} is connected to {oracle_name}")
            orc_conns.append(conn)

            # Connect to GCS
            port = BasePort.GCS + 10 * (sysid - 1)
            conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
            conn.wait_heartbeat()
            gcs_name = gcs_of[sysid]
            print(f"🔗 UAV logic {sysid} is connected to {gcs_name}")
            gcs_conns[gcs_name].append(conn)
        oracle = Oracle(orc_conns, name=oracle_name)
        gcss: list[GCS] = []
        for name, conns in gcs_conns.items():
            gcss.append(GCS(conns, name))
        return oracle, gcss

    def _add_vehicle_cmd_fn(self, _i: int) -> str:
        """Optional hook to additional command-line args."""
        return ""

    def _launch_visualizer(self) -> None:
        """Optional hook to launch a visual simulator or GUI application."""
        print("🙈 Running without visualization.")

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
