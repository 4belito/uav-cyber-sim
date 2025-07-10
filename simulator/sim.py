"""
Simulation script that launches the full setup:
1. ArduPilot instances for each vehicle.
2. Logic process for each vehicle.
3. Optionally a simulator (None, QGroundControl, or Gazebo).
"""

import json
import socket
from concurrent import futures
from itertools import product
from pathlib import Path
from typing import Literal, TypeVar

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import (
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
    ENV_CMD_ARP,
    ENV_CMD_PYT,
    LOGS_PATH,
    VEH_PARAMS_PATH,
    BasePort,
)
from helpers import create_process, reset_folder
from mavlink.customtypes.connection import MAVConnection
from mavlink.util import save_mission
from oracle import Oracle
from simulator.visualizer import Visualizer

from .QGroundControl.config import Missions

SimProcess = Literal["launcher", "veh", "logic", "proxy", "gcs"]

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
        missions: Missions,
        gcs_sysids: dict[str, list[int]],
        terminals: list[SimProcess] = [],
        supress_output: list[SimProcess] = ["launcher"],
        verbose: int = 1,
    ):
        self.visuals = visualizers
        self.terminals: dict[SimProcess, bool] = dict.fromkeys(terminals, True)
        self.supress: dict[SimProcess, bool] = dict.fromkeys(supress_output, True)
        self.n_vehs = visualizers[0].config.n_vehicles
        self.verbose = verbose
        self.port_offsets: list[int] = []
        self.gcs_sysids = gcs_sysids
        self.missions = missions

    def launch(self) -> Oracle:
        """Launch vehicle instances and the optional simulator."""
        # Simulator.save_gcs_sysids(gcs_sysids)
        reset_folder(DATA_PATH)
        self.save_missions()
        self.port_offsets = self._find_port_offsets()
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        for visual in self.visuals:
            if not visual.delay:
                visual.launch(self.port_offsets, self.verbose)
        oracle = self._launch_vehicles()
        for visual in self.visuals:
            if visual.delay:
                visual.launch(self.port_offsets, self.verbose)

        return oracle

    def save_missions(self):
        """Save the missions for all the vehicles."""
        for i, mission in enumerate(self.missions):
            traj = [wp.pos for wp in mission.traj]
            save_mission(
                path=DATA_PATH / f"mission_{i + 1}.waypoints",
                poses=traj,
                delay=mission.delay,
            )

    def _save_logic_configs(self, folder_name: Path):
        """Save the logic configurations for each UAV."""
        for i in range(self.n_vehs):
            sysid = i + 1
            logic_config = {
                "sysid": sysid,
                "port_offset": self.port_offsets[i],
            }
            config_path = folder_name / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self, folder_name: Path):
        for gcs_name, sysids in self.gcs_sysids.items():
            gcs_config = {
                "name": gcs_name,
                "uavs": [
                    {"sysid": sysid, "port_offset": self.port_offsets[sysid - 1]}
                    for sysid in sysids
                ],
            }
            config_path = folder_name / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _launch_vehicles(self) -> Oracle:
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

        oracle = Oracle(orc_conns, name=self.oracle_name, verbose=self.verbose)
        for gcs_name in self.gcs_sysids.keys():
            gcs_cmd = f'python3 gcs.py --name "{gcs_name}" --verbose {self.verbose}'
            p = create_process(
                gcs_cmd,
                after="exec bash",
                visible=self.terminals.get("gcs", False),
                suppress_output=self.supress.get("gcs", False),
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            if self.verbose:
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
        p = create_process(
            veh_cmd,
            after="exec bash",
            visible=self.terminals.get("launcher", False),
            suppress_output=self.supress.get("launcher", False),
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        if self.verbose:
            print(f"🚀 ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        logic_cmd = f"python3 logic.py --sysid {sysid} " f"--verbose {self.verbose} "
        p = create_process(
            logic_cmd,
            after="exec bash",
            visible=self.terminals.get("logic", False),
            suppress_output=self.supress.get("logic", False),
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        if self.verbose:
            print(f"🚀 UAV logic for vehicle {sysid} launched (PID {p.pid})")

        proxy_cmd = (
            f"python3 proxy.py --sysid {sysid} "
            f"--port-offset={self.port_offsets[i]} "
            f"--verbose {self.verbose}"
        )
        p = create_process(
            proxy_cmd,
            after="exec bash",
            visible=self.terminals.get("proxy", False),
            suppress_output=self.supress.get("proxy", False),
            title=f"Proxy: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        if self.verbose:
            print(f"🚀 Proxy for vehicle {sysid} launched (PID {p.pid})")

        ## Connect to oracle
        port = BasePort.ORC + self.port_offsets[i]
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        if self.verbose:
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
