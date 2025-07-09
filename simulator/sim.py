"""
Simulation script that launches the full setup:
1. ArduPilot instances for each vehicle.
2. Logic process for each vehicle.
3. Optionally a simulator (None, QGroundControl, or Gazebo).
"""

import platform
import socket
from concurrent import futures
from itertools import product
from subprocess import Popen
from typing import Literal, TypeVar

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
from oracle import Oracle
from simulator.visualizer import Visualizer

Terminals = Literal["launcher", "veh", "logic", "proxy", "gcs"]

V = TypeVar("V")  # Vehicle type


class Simulator:
    """
    Base simulator class to manage UAV vehicle processes and optional external
    simulators.

    Args:
        name (VisualizerName): Type of simulator to use.
        offsets: Spawn offsets for each UAV.
        plans: Mission plans for each UAV.

    """

    oracle_name: str = "Oracle ⚪"

    def __init__(
        self,
        visualizers: list[Visualizer[V]],
        terminals: list[Terminals] = [],
        verbose: int = 1,
    ):
        self.visuals = visualizers
        self.terminals: dict[Terminals, bool] = dict.fromkeys(terminals, True)
        self.n_vehs = visualizers[0].config.n_vehicles
        self.verbose = verbose
        self.port_offsets: list[int] = []

    def launch(self, gcs_sysids: dict[str, list[int]]) -> Oracle:
        """Launch vehicle instances and the optional simulator."""
        # Simulator.save_gcs_sysids(gcs_sysids)
        self.port_offsets = self._find_port_offsets()
        for visual in self.visuals:
            if not visual.delay:
                visual.launch(self.port_offsets, self.verbose)
        oracle = self._launch_vehicles(gcs_sysids)
        for visual in self.visuals:
            if visual.delay:
                visual.launch(self.port_offsets, self.verbose)

        return oracle

    # @staticmethod
    # def save_gcs_sysids(gcs_sysids: dict[str, list[int]]):
    #     for gcs_name, sysids in gcs_sysids.items():
    #         with open(f"sysids_{gcs_name}.txt", "w") as f:
    #             f.write(str(sysids))

    def _launch_vehicles(self, gcs_sysids: dict[str, list[int]]) -> Oracle:
        """Launch ArduPilot and logic processes for each UAV."""
        # with futures.ThreadPoolExecutor() as executor:
        #     orc_conns = list(executor.map(self._launch_uav, range(self.n_vehs)))

        args = list(product(range(self.n_vehs), range(len(self.visuals))))

        with futures.ThreadPoolExecutor() as executor:
            futures_list = [executor.submit(self._launch_uav, i, j) for i, j in args]
            orc_conns: dict[int, MAVConnection] = {}
            for f in futures_list:
                conn = f.result()
                orc_conns[conn.target_system] = conn

        oracle = Oracle(orc_conns, name=self.oracle_name)
        for gcs_name, sysids in gcs_sysids.items():
            gcs_cmd = (
                f'python3 gcs.py --name "{gcs_name}" '
                f'--sysid "{sysids}" '
                f'--port-offsets "'
                f'{[self.port_offsets[sysid - 1] for sysid in sysids]}" '
            )
            p = Simulator.create_process(
                gcs_cmd,
                after="exec bash",
                visible=self.terminals.get("gcs", False),
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            print(f"🚀 GCS {gcs_name} launched (PID {p.pid})")
        return oracle

    def _launch_uav(self, i: int, j: int):
        sysid = i + 1
        veh_cmd = (
            f"python3 {ARDUPILOT_VEHICLE_PATH}"
            f" -v ArduCopter -I{i} --sysid {sysid} --no-rebuild"
            f" --use-dir={LOGS_PATH} --add-param-file {VEH_PARAMS_PATH}"
            f" --no-mavproxy"
            f" --port-offset={self.port_offsets[i]}"
            + (" --terminal" if self.terminals.get("veh", False) else "")
        )
        veh_cmd += self.visuals[j].add_vehicle_cmd(i)
        p = Simulator.create_process(
            veh_cmd,
            after="exec bash",
            visible=self.terminals.get("launcher", False),
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        print(f"🚀 ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        logic_cmd = (
            f"python3 logic.py --sysid {sysid} "
            f"--port-offset={self.port_offsets[i]} "
            f"--verbose {self.verbose} "
        )
        p = Simulator.create_process(
            logic_cmd,
            after="exec bash",
            visible=self.terminals.get("logic", False),
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        print(f"🚀 UAV logic for vehicle {sysid} launched (PID {p.pid})")

        proxy_cmd = (
            f"python3 proxy.py --sysid {sysid} "
            f"--port-offset={self.port_offsets[i]} "
            f"--verbose {self.verbose}"
        )
        p = Simulator.create_process(
            proxy_cmd,
            after="exec bash",
            visible=self.terminals.get("proxy", False),
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

    def _find_port_offsets(self):
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
        while len(offsets) < self.n_vehs:
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

    @staticmethod
    def create_process(
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
                        "--geometry=71x10",  # width=100 cols, height=30 rows
                        "--",
                    ]
                    + bash_cmd
                )
            raise OSError("Unsupported OS for visible terminal mode.")
        return Popen(bash_cmd)
