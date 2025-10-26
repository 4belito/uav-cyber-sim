"""
Launches multi-UAV simulation with ArduPilot SITL, logic, proxies,
and optional visualization.
"""

import json
import logging
import socket
from pathlib import Path
from typing import Callable, Generic, Literal

from config import (
    ARDU_LOGS_PATH,
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    VEH_PARAMS_PATH,
    BasePort,
)
from helpers import create_process, setup_logging
from oracle import Oracle
from simulator.visualizer import Visualizer

from .vehicle import SimVehicle, SimVehicles, V

SimProcess = Literal["launcher", "veh", "logic", "proxy", "gcs"]


class Simulator(Generic[V]):
    """
    Manages a full multi-UAV simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"

    def __init__(
        self,
        # visualization
        visualizer: Visualizer[V],
        terminals: list[SimProcess] = [],
        supress_output: list[SimProcess] = ["launcher"],
        verbose: int = 1,
        # oracle
        transmission_range: int = 100,  # meters for inter-UAV communication
    ):
        self.visualizer = visualizer
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = set(terminals)
        self.suppress = set(supress_output)
        self.sysids: list[int] = []
        self.gcs_names: list[str] = []
        self.n_vehs = len(self.sysids)
        self.n_gcss = len(self.gcs_names)
        self.verbose = verbose
        self.gcs_sysids: dict[str, list[int]] = {}
        self.vehs: SimVehicles = []
        self.logic_cmd: Callable[[int, str, int], str] = (
            lambda _, config_path, verbose: (
                f'python3 logic.py --config-path "{config_path}" --verbose {verbose} '
            )
        )
        self.gcs_cmd: Callable[[int, str, int], str] = lambda _, config_path, verbose: (
            f'python3 gcs.py --config-path "{config_path}" --verbose {verbose}'
        )
        self.transmission_range = transmission_range  # meters

        setup_logging(self.oracle_name, verbose=verbose, console_output=True)
        logging.debug(
            (
                f"simulator initialized with {self.n_vehs} vehicles "
                f"and {self.n_gcss} GCSs"
            )
        )

    def add_vehicle(self, vehicle: SimVehicle):
        """Add a vehicle to the simulation."""
        self.vehs.append(vehicle)
        self.sysids.append(vehicle.sysid)
        if vehicle.gcs_name not in self.gcs_sysids:
            self.gcs_sysids[vehicle.gcs_name] = []
            self.gcs_names.append(vehicle.gcs_name)
        self.gcs_sysids[vehicle.gcs_name].append(vehicle.sysid)
        self.n_vehs = len(self.sysids)
        self.n_gcss = len(self.gcs_names)
        self.visualizer.add_vehicle(vehicle)

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        # self.save_missions()
        self.uav_port_offsets = self._find_uav_port_offsets()
        self.gcs_port_offsets = self._find_gcs_port_offsets()
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        if not self.visualizer.delay:
            self.visualizer.launch(self.uav_port_offsets)
        self._launch_gcses()
        oracle = self._launch_oracle()
        if self.visualizer.delay:
            self.visualizer.launch(self.uav_port_offsets)
        return oracle

    def show(self):
        """
        Render a static preview of the configured simulation
        before launch.
        """
        self.visualizer.show()

    def _launch_gcses(self):
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

    def _launch_oracle(self) -> Oracle:
        uav_port_offsets = dict(zip(self.sysids, self.uav_port_offsets))
        gcs_port_offsets = dict(zip(self.gcs_names, self.gcs_port_offsets))
        return Oracle(
            self.gra_origin,
            uav_port_offsets,
            gcs_port_offsets,
            self.gcs_sysids,
            transmission_range=self.transmission_range,
        )

    def _save_logic_configs(self, folder_name: Path):
        """Save the logic configurations for each UAV."""
        for i, sysid in enumerate(self.sysids):
            logic_config = {
                "sysid": sysid,
                "gra_origin_dict": {
                    "lat": self.gra_origin.lat,
                    "lon": self.gra_origin.lon,
                    "alt": self.gra_origin.alt,
                },
                "port_offset": self.uav_port_offsets[i],
                "navegation_speed": 5,
            }
            config_path = folder_name / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self, folder_name: Path):
        n = 0
        for i, (gcs_name, sysids) in enumerate(self.gcs_sysids.items()):
            gcs_config = {
                "name": gcs_name,
                "port_offset": self.gcs_port_offsets[i],
                "uavs": [
                    {
                        "sysid": sysid,
                        "port_offset": self.uav_port_offsets[j],
                        "ardupilot_cmd": (
                            f"python3 {ARDUPILOT_VEHICLE_PATH}"
                            f" -v ArduCopter -I{j} --sysid {sysid} --no-rebuild"
                            f" --use-dir={ARDU_LOGS_PATH}"
                            f" --add-param-file {VEH_PARAMS_PATH}"
                            f" --no-mavproxy"
                            f" --port-offset={self.uav_port_offsets[j]}"
                            + (" --terminal" if "veh" in self.terminals else "")
                            + self.visualizer.add_vehicle_cmd(j)
                        ),
                        "logic_cmd": self.logic_cmd(
                            sysid,
                            str(DATA_PATH / f"logic_config_{sysid}.json"),
                            self.verbose,
                        ),
                        "proxy_cmd": (
                            f"python3 proxy.py --sysid {sysid} "
                            f"--port-offset={self.uav_port_offsets[j]} "
                            f"--verbose {self.verbose}"
                        ),
                    }
                    for j, sysid in enumerate(sysids, start=n)
                ],
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }
            config_path = folder_name / f"gcs_config_{i + 1}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)
            n += len(sysids)

    # TODO: Check why BasePort.GCS is in find_uav_port_offset
    # and no in find_gcs_port_offset
    def _find_uav_port_offsets(self):
        base_ports = [
            BasePort.ARP,
            BasePort.GCS,
            BasePort.QGC,
            BasePort.LOG,
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
