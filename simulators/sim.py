"""
Simulation script that launches the full setup:
1. ArduPilot instances for each vehicle.
2. Logic process for each vehicle.
3. Optionally a simulator (None, QGroundControl, or Gazebo).
"""

import platform
import socket
from concurrent import futures
from enum import Enum
from pathlib import Path
from subprocess import Popen

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import (
    ARDUPILOT_VEHICLE_PATH,
    ENV_CMD_ARP,
    ENV_CMD_PYT,
    LOGS_PATH,
    VEH_PARAMS_PATH,
    BasePort,
)
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENUPose, ENUPoses
from oracle import Oracle
from plan import Plan, Plans

from .gazebo.config import ConfigGazebo
from .QGroundControl.config import ConfigQGC


class VisualizerName(str, Enum):
    """Simulator type options used to configure the simulation environment."""

    NONE = "none"
    QGROUND = "qgroundcontrol"
    GAZEBO = "gazebo"

    def __str__(self):
        return str(self.value)


# TODO: Improve Simulatior Class design


class Simulator:
    """
    Base simulator class to manage UAV vehicle processes and optional external
    simulators.

    Args:
        name (VisualizerName): Type of simulator to use.
        offsets: Spawn offsets for each UAV.
        plans: Mission plans for each UAV.

    """

    def __init__(
        self,
        name: VisualizerName = VisualizerName.NONE,
        offsets: ENUPoses | None = None,
        plans: Plans | None = None,
        visible_terminals: bool = True,
        oracle_name: str = "Oracle ⚪",
        delay_visualizer: bool = False,
        verbose: int = 1,
    ):
        self.name = name
        self.config: ConfigGazebo | ConfigQGC | None = None
        self.offsets = offsets or [ENUPose(0, 0, 0, 0)]
        self.n_uavs: int = len(self.offsets)
        self.ardu_path: Path = ARDUPILOT_VEHICLE_PATH
        self.plans: Plans = plans or [Plan.basic()]
        self.visible_terminals = visible_terminals
        self.oracle_name = oracle_name
        self.delay_visualizer = delay_visualizer
        self.verbose = verbose

    def launch(self, gcs_sysids: dict[str, list[int]]) -> Oracle:
        """Launch vehicle instances and the optional simulator."""
        self.port_offsets = self.find_port_offsets()
        if self.delay_visualizer:
            oracle = self.launch_vehicles(gcs_sysids)
            self._launch_visualizer()
        else:
            self._launch_visualizer()
            oracle = self.launch_vehicles(gcs_sysids)
        return oracle

    def launch_vehicles(self, gcs_sysids: dict[str, list[int]]) -> Oracle:
        """Launch ArduPilot and logic processes for each UAV."""
        with futures.ThreadPoolExecutor() as executor:
            orc_conns = list(executor.map(self._launch_uav, range(self.n_uavs)))

        oracle = Oracle(orc_conns, name=self.oracle_name)
        for gcs_name, sysids in gcs_sysids.items():
            gcs_cmd = (
                f'python3 gcs.py --name "{gcs_name}" '
                f'--sysid "{sysids}" '
                f'--port-offsets "{[self.port_offsets[sysid - 1] for sysid in sysids]}"'
            )
            p = self.create_process(
                gcs_cmd,
                after="exec bash",
                visible=self.visible_terminals,
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            print(f"🚀 GCS {gcs_name} launched (PID {p.pid})")
        return oracle

    def _launch_uav(self, i: int):
        sysid = i + 1
        veh_cmd = (
            f"python3 {self.ardu_path}"
            f" -v ArduCopter -I{i} --sysid {sysid} --no-rebuild"
            f" --use-dir={LOGS_PATH} --add-param-file {VEH_PARAMS_PATH}"
            f" --no-mavproxy"
            f" --port-offset={self.port_offsets[i]}"
            + (" --terminal" if self.visible_terminals else "")
        )
        veh_cmd += self._add_vehicle_cmd_fn(i)
        p = self.create_process(
            veh_cmd,
            after="exec bash",
            visible=self.visible_terminals,
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        print(f"🚀 ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        logic_cmd = (
            f"python3 logic.py --sysid {sysid} "
            f"--port-offset={self.port_offsets[i]} "
            f"--verbose {self.verbose}"
        )
        p = self.create_process(
            logic_cmd,
            after="exec bash",
            visible=self.visible_terminals,
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        print(f"🚀 UAV logic for vehicle {sysid} launched (PID {p.pid})")

        proxy_cmd = (
            f"python3 proxy.py --sysid {sysid} "
            f"--port-offset={self.port_offsets[i]} "
            f"--verbose {self.verbose}"
        )
        p = self.create_process(
            proxy_cmd,
            after="exec bash",
            visible=self.visible_terminals,
            title=f"Proxy: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        print(f"🚀 Proxy for vehicle {sysid} launched (PID {p.pid})")
        print(f"🔗 UAV logic {sysid} is connected to Ardupilot SITL vehicle {sysid}")

        ## Connect to oracle
        port = BasePort.ORC + self.port_offsets[i]
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        print(f"🔗 UAV logic {sysid} is connected to {self.oracle_name}")
        return conn

    def find_port_offsets(self):
        """Find available port offsets for each UAV to avoid conflicts."""
        base_ports = [
            BasePort.ARP,
            BasePort.GCS,
            BasePort.ORC,
            BasePort.QGC,
            BasePort.VEH,
        ]
        unit_offset = 10
        offsets = list[int]()

        cur_offset = 0
        while len(offsets) < self.n_uavs:
            for base_port in base_ports:
                port = base_port + cur_offset
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.01)
                try:
                    s.bind(("127.0.0.1", port))
                    s.close()
                except Exception:
                    break
            else:
                offsets.append(cur_offset)
                # print(f"Found offset {len(offsets)} - {cur_offset}")
            cur_offset += unit_offset
        return offsets

    def _add_vehicle_cmd_fn(self, _i: int) -> str:
        """Add optional command-line arguments for the vehicle."""
        return ""

    def _launch_visualizer(self) -> None:
        """Launch a visual simulator or GUI application if configured."""
        print("🙈 Running without visualization.")

    def create_process(
        self,
        cmd: str,
        after: str = "exit",
        visible: bool = True,
        title: str = "Terminal",
        env_cmd: str | None = None,
    ) -> Popen[bytes]:
        """Launch a subprocess, optionally in a visible terminal."""
        bash_cmd = [
            "bash",
            "-c",
            (f"{env_cmd}; " if env_cmd else "") + f"{cmd}; {after}",
        ]
        if visible:
            if platform.system() == "Linux":
                return Popen(
                    [
                        "gnome-terminal",
                        "--title",
                        title,
                        "--",
                    ]
                    + bash_cmd
                )
            raise OSError("Unsupported OS for visible terminal mode.")
        return Popen(bash_cmd)

    def __repr__(self) -> str:
        return f"name: '{self.name}'\noffsets: {self.offsets}\nconfig: {self.config}"
