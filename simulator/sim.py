"""
Launches multi-UAV simulation with ArduPilot SITL, logic, proxies,
and optional visualization.
"""

import json
import logging
import socket
from concurrent import futures
from pathlib import Path
from typing import Callable, Literal, TypeVar

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import (
    ARDU_LOGS_PATH,
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    VEH_PARAMS_PATH,
    BasePort,
)
from helpers import create_process, setup_logging
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
        gcs_names: list[str],
        gcs_sysids: list[list[int]],
        logic_cmd: Callable[[int, str, int], str] = lambda _, config_path, verbose: (
            f'python3 logic.py --config-path "{config_path}" --verbose {verbose} '
        ),
        gcs_cmd: Callable[[int, str, int], str] = lambda _, config_path, verbose: (
            f'python3 gcs.py --config-path "{config_path}" --verbose {verbose}'
        ),
        monitored_mission_items: list[list[int]] | None = None,
        # visualization
        terminals: list[SimProcess] = [],
        supress_output: list[SimProcess] = ["launcher"],
        verbose: int = 1,
    ):
        self.visuals = visualizers
        self.terminals = set(terminals)
        self.suppress = set(supress_output)
        self.n_vehs = visualizers[0].config.n_vehicles
        self.n_gcss = len(gcs_names)
        self.verbose = verbose
        self.gcs_names = gcs_names
        self.gcs_sysids = gcs_sysids
        self.missions = missions
        self.logic_cmd = logic_cmd
        self.gcs_cmd = gcs_cmd
        self.monitored_items = monitored_mission_items or [
            list(range(1, mission.n_items - 1)) for mission in missions
        ]
        setup_logging(self.oracle_name, verbose=verbose, console_output=True)
        logging.debug(
            (
                f"simulator initialized with {self.n_vehs} vehicles "
                f"and {self.n_gcss} GCSs"
            )
        )

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        self.save_missions()
        self.uav_port_offsets = self._find_uav_port_offsets()
        self.gcs_port_offsets = self._find_gcs_port_offsets()
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        for visual in self.visuals:
            if not visual.delay:
                visual.launch(self.uav_port_offsets)
        oracle = self._launch_gcses()
        for visual in self.visuals:
            if visual.delay:
                visual.launch(self.uav_port_offsets)

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
                "port_offset": self.uav_port_offsets[i],
                "monitored_items": self.monitored_items[i],
            }
            config_path = folder_name / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self, folder_name: Path):
        for i, (gcs_name, sysids) in enumerate(zip(self.gcs_names, self.gcs_sysids)):
            gcs_config = {
                "name": gcs_name,
                "port_offset": self.gcs_port_offsets[i],
                "uavs": [
                    {
                        "sysid": sysid,
                        "port_offset": self.uav_port_offsets[sysid - 1],
                        "ardupilot_cmd": (
                            f"python3 {ARDUPILOT_VEHICLE_PATH}"
                            f" -v ArduCopter -I{sysid - 1} --sysid {sysid} --no-rebuild"
                            f" --use-dir={ARDU_LOGS_PATH}"
                            f" --add-param-file {VEH_PARAMS_PATH}"
                            f" --no-mavproxy"
                            f" --port-offset={self.uav_port_offsets[sysid - 1]}"
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
                            f"--port-offset={self.uav_port_offsets[sysid - 1]} "
                            f"--verbose {self.verbose}"
                        ),
                    }
                    for sysid in sysids
                ],
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }
            config_path = folder_name / f"gcs_config_{i + 1}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _launch_gcses(self) -> Oracle:
        """Launch each GCS process and create an Oracle instance."""
        for i, gcs_name in enumerate(self.gcs_names):
            gcsid = i + 1
            gcs_cmd = self.gcs_cmd(
                gcsid, str(DATA_PATH / f"gcs_config_{gcsid}.json"), self.verbose
            )
            p = create_process(
                gcs_cmd,
                after="exec bash",
                visible="gcs" in self.terminals,
                suppress_output="gcs" in self.suppress,
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            logging.info(f"🚀 GCS {gcs_name} launched (PID {p.pid})")

        # Wait for GCS processes to launch their vehicles before connecting
        import time

        logging.info("⏳ Waiting for GCS processes to launch vehicles...")
        time.sleep(5)  # Give GCS processes time to launch all their vehicles
        logging.info("🔗 Starting Oracle connections to vehicles...")

        ## Connect to oracle
        with futures.ThreadPoolExecutor() as executor:
            orc_conns = dict(
                zip(
                    range(1, self.n_vehs + 1),
                    executor.map(self._connect_to_vehicle, range(self.n_vehs)),
                )
            )
        oracle = Oracle(
            orc_conns,
            uav_port_offsets={
                i + 1: offset for i, offset in enumerate(self.uav_port_offsets)
            },
            gcs_port_offsets={
                name: offset
                for name, offset in zip(self.gcs_names, self.gcs_port_offsets)
            },
            gcs_sysids={
                name: sysids for name, sysids in zip(self.gcs_names, self.gcs_sysids)
            },
        )
        return oracle

    def _connect_to_vehicle(self, i: int) -> MAVConnection:
        """Connect to a UAV through MAVLink."""
        port = BasePort.ORC + self.uav_port_offsets[i]
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}", source_system=i + 1)  # type: ignore
        conn.wait_heartbeat()
        logging.info(f"🔗 UAV logic {i + 1} is connected to {self.oracle_name}")
        return conn

    def _find_uav_port_offsets(self):
        base_ports = [
            BasePort.ARP,
            BasePort.GCS,
            BasePort.ORC,
            BasePort.QGC,
            BasePort.VEH,
            BasePort.RID_UP,
            BasePort.RID_DOWN,
            BasePort.RID_DATA,
        ]
        return self._find_port_offsets(base_ports, self.n_vehs)

    # excluded_offsets=[160]

    def _find_gcs_port_offsets(self) -> list[int]:
        base_ports = [BasePort.GCS_ZMQ]
        return self._find_port_offsets(base_ports, self.n_gcss)

    def _find_port_offsets(
        self,
        base_ports: list[BasePort],
        n_ports: int,
        unit_offset: int = 10,
        excluded_offsets: list[int] = [],
    ) -> list[int]:
        """Find available port offsets for each UAV to avoid conflicts."""
        offsets = list[int]()

        cur_offset = 0
        while len(offsets) < n_ports:
            if cur_offset in excluded_offsets:
                cur_offset += 10
                continue
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
            cur_offset += unit_offset
        return offsets
