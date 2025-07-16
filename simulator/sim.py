"""
Launches multi-UAV simulation with ArduPilot SITL, logic, proxies,
and optional visualization.
"""

import json
import socket
from concurrent import futures
from pathlib import Path
from typing import Callable, Literal, TypeVar

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import (
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
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
    Manages a full multi-UAV simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"

    def __init__(
        self,
        visualizers: list[Visualizer[V]],
        missions: Missions,
        gcs_sysids: dict[str, list[int]],
        logic_cmd: Callable[[int, str, int], str] = lambda _, config_path, verbose: (
            f'python3 logic.py --config-path "{config_path}" --verbose {verbose} '
        ),
        gcs_cmd: Callable[[str, str, int], str] = lambda _, config_path, verbose: (
            f'python3 gcs.py --config-path "{config_path}" --verbose {verbose}'
        ),
        monitored_mission_items: list[list[int]] | None = None,
        # visualization
        terminals: list[SimProcess] = [],
        supress_output: list[SimProcess] = ["launcher"],
        verbose: int = 1,
    ):
        self.visuals = visualizers
        self.terminals: set[SimProcess] = set(terminals)
        self.suppress: set[SimProcess] = set(supress_output)
        self.n_vehs = visualizers[0].config.n_vehicles
        self.verbose = verbose
        self.port_offsets: list[int] = []
        self.gcs_sysids = gcs_sysids
        self.missions = missions
        self.logic_cmd = logic_cmd
        self.gcs_cmd = gcs_cmd
        self.monitored_items = monitored_mission_items or [
            list(range(1, mission.n_items - 1)) for mission in missions
        ]

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        reset_folder(DATA_PATH)
        self.save_missions()
        self.port_offsets = self._find_port_offsets()
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        for visual in self.visuals:
            if not visual.delay:
                visual.launch(self.port_offsets, self.verbose)
        oracle = self._launch_gcses()
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
                "monitored_items": self.monitored_items[i],
            }
            config_path = folder_name / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self, folder_name: Path):
        for gcs_name, sysids in self.gcs_sysids.items():
            gcs_config = {
                "name": gcs_name,
                "uavs": [
                    {
                        "sysid": sysid,
                        "port_offset": self.port_offsets[sysid - 1],
                        "ardupilot_cmd": (
                            f"python3 {ARDUPILOT_VEHICLE_PATH}"
                            f" -v ArduCopter -I{sysid - 1} --sysid {sysid} --no-rebuild"
                            f" --use-dir={LOGS_PATH} --add-param-file {VEH_PARAMS_PATH}"
                            f" --no-mavproxy"
                            f" --port-offset={self.port_offsets[sysid - 1]}"
                            + (" --terminal" if "veh" in self.terminals else "")
                            + self.visuals[0].add_vehicle_cmd(sysid - 1)
                        ),
                        "logic_cmd": self.logic_cmd(
                            sysid,
                            str(DATA_PATH / f"logic_config_{sysid}.json"),
                            self.verbose,
                        ),
                        "proxy_cmd": (
                            f"python3 proxy.py --sysid {sysid} "
                            f"--port-offset={self.port_offsets[sysid - 1]} "
                            f"--verbose {self.verbose}"
                        ),
                    }
                    for sysid in sysids
                ],
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }
            config_path = folder_name / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _launch_gcses(self) -> Oracle:
        """Launch each GCS process and create an Oracle instance."""
        for gcs_name in self.gcs_sysids.keys():
            gcs_cmd = self.gcs_cmd(
                gcs_name, str(DATA_PATH / f"gcs_config_{gcs_name}.json"), self.verbose
            )
            p = create_process(
                gcs_cmd,
                after="exec bash",
                visible="gcs" in self.terminals,
                suppress_output="gcs" in self.suppress,
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            if self.verbose:
                print(f"🚀 GCS {gcs_name} launched (PID {p.pid})")

        ## Connect to oracle
        with futures.ThreadPoolExecutor() as executor:
            orc_conns = dict(
                zip(
                    range(self.n_vehs),
                    executor.map(self._connect_to_vehicle, range(self.n_vehs)),
                )
            )
        oracle = Oracle(orc_conns, name=self.oracle_name, verbose=self.verbose)
        return oracle

    def _connect_to_vehicle(self, i: int) -> MAVConnection:
        """Connect to a UAV through MAVLink."""
        port = BasePort.ORC + self.port_offsets[i]
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        if self.verbose:
            print(f"🔗 UAV logic {i + 1} is connected to {self.oracle_name}")
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
