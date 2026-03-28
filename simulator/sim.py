"""
Launches multi-UAV simulation with ArduPilot SITL, logic, and optional
visualization.
"""

import json
import logging
import socket
from pathlib import Path
from typing import Generic

from simulator.config import (
    ARDU_LOGS_PATH,
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    VEH_PARAMS_PATH,
    BasePort,
)
from simulator.configs.gcs import UAVGCSConfig
from simulator.entities import SimGCS, SimVehicle, VehT
from simulator.helpers.processes import SimProcess, create_process
from simulator.helpers.setup_log import setup_logging
from simulator.oracle import Oracle
from simulator.visualizer import Visualizer


class Simulator(Generic[VehT]):
    """
    Manages a full multi-UAV simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"

    def __init__(
        self,
        visualizer: Visualizer[VehT],
        terminals: list[SimProcess] = [],
        suppress_output: list[SimProcess] = [
            SimProcess.ARDUPILOT,
            SimProcess.ADSB_SOCAT,
        ],
        verbose: int = 1,
        # oracle
        transmission_range: int = 100,  # meters for inter-UAV communication
    ):
        self.visualizer = visualizer
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = set(terminals)
        self.suppress = set(suppress_output)
        self.vehs: dict[int, SimVehicle] = {}
        self.gcs: dict[str, SimGCS] = {}
        self.verbose = verbose
        self.instance = 0
        self.transmission_range = transmission_range  # meters

        setup_logging(self.oracle_name, verbose=verbose, console_output=True)

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        uav_port_offsets = self._find_uav_port_offsets()
        gcs_port_offsets = self._find_gcs_port_offsets()
        for sysid, offset in zip(sorted(self.vehs), uav_port_offsets):
            self.vehs[sysid].port_offset = offset
        for gcs_name, offset in zip(sorted(self.gcs), gcs_port_offsets):
            self.gcs[gcs_name].port_offset = offset
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        self.visualizer.launch(uav_port_offsets)
        self._launch_gcses()
        return Oracle(
            self.gra_origin,
            self.vehs,
            self.gcs,
            transmission_range=self.transmission_range,
        )

    def add_vehicle(self, vehicle: SimVehicle):
        """Add a vehicle to the simulation."""
        vehicle.instance = self.instance
        self.instance += 1
        self.vehs[vehicle.sysid] = vehicle
        if vehicle.gcs_name not in self.gcs:
            self.gcs[vehicle.gcs_name] = SimGCS(name=vehicle.gcs_name)
        self.gcs[vehicle.gcs_name].sysids.append(vehicle.sysid)
        self.visualizer.add_vehicle(vehicle)

    def remove_vehicle(self, sysid: int) -> bool:
        """Remove a vehicle by system ID."""
        if sysid in self.vehs:
            del self.vehs[sysid]
            self.visualizer.remove_vehicle(sysid)
            return True
        return False

    def show(self):
        """
        Render a static preview of the configured simulation
        before launch.
        """
        self.visualizer.show()

    def _launch_gcses(self):
        """Launch each GCS process and create an Oracle instance."""
        for gcs_name in self.gcs:
            gcs_config_path = DATA_PATH / f"gcs_config_{gcs_name}.json"
            gcs_cmd = (
                f'python3 -m simulator.gcs --config-path "{gcs_config_path}"'
                f" --verbose {self.verbose}"
            )
            p = create_process(
                gcs_cmd,
                after="exec bash",
                visible=SimProcess.GCS in self.terminals,
                suppress_output=SimProcess.GCS in self.suppress,
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            logging.info(f"🚀 GCS {gcs_name} launched (PID {p.pid})")

    def _save_logic_configs(self, folder_name: Path):
        """Save the logic configurations for each UAV."""
        for sysid, veh in self.vehs.items():
            logic_config = {
                "sysid": sysid,
                "gra_origin_dict": {
                    "lat": self.gra_origin.lat,
                    "lon": self.gra_origin.lon,
                    "alt": self.gra_origin.alt,
                },
                "port_offset": veh.port_offset,
                "plan_spec": veh.plan.get_spec().to_dict(),
            }
            config_path = folder_name / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self, folder_name: Path):
        for gcs_name, gcs in self.gcs.items():
            gcs_config = {
                "name": gcs_name,
                "port_offset": gcs.port_offset,
                "uavs": [self._build_uav_config(sysid) for sysid in gcs.sysids],
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }

            config_path = folder_name / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _find_uav_port_offsets(self):
        base_ports = [
            BasePort.ARP,
            BasePort.ARP2,
            BasePort.ARP3,
            BasePort.RID_UP,
            BasePort.RID_DOWN,
            BasePort.RID_DATA,
        ]
        return self._find_port_offsets(base_ports, len(self.vehs))

    def _find_gcs_port_offsets(self) -> list[int]:
        base_ports = [BasePort.GCS_ZMQ, BasePort.GCS]
        return self._find_port_offsets(base_ports, len(self.gcs))

    def _find_port_offsets(
        self,
        base_ports: list[BasePort],
        n_ports: int,
        unit_offset: int = 10,
    ) -> list[int]:
        """Find available port offsets for each UAV to avoid conflicts."""
        offsets: list[int] = []

        cur_offset = 0
        while len(offsets) < n_ports:
            for base_port in base_ports:
                port = base_port + cur_offset
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.01)
                    try:
                        s.bind(("0.0.0.0", port))
                    except Exception:
                        break
            else:
                offsets.append(cur_offset)
            cur_offset += unit_offset
        return offsets

    def _build_uav_config(self, sysid: int) -> UAVGCSConfig:
        veh = self.vehs[sysid]

        port_offset = veh.port_offset_required
        inst = int(port_offset / 10)

        param_file = ARDU_LOGS_PATH / f"uav_{sysid}"
        param_file.mkdir(parents=True, exist_ok=True)

        sitl_args = (
            f"--serial5=uart:/tmp/adsb_{sysid}_ardupilot:57600"
            f"{self.visualizer.add_sitl_args()}"
        )

        logic_config_path = str(DATA_PATH / f"logic_config_{sysid}.json")

        uav_config: UAVGCSConfig = {
            "sysid": sysid,
            "port_offset": port_offset,
            "ardupilot_cmd": (
                f"python3 {ARDUPILOT_VEHICLE_PATH}"
                f" -v ArduCopter -I{inst} --sysid {sysid} --no-rebuild"
                f' -A "{sitl_args}"'
                f" --use-dir={param_file}"
                f" --add-param-file {VEH_PARAMS_PATH}"
                f" --no-mavproxy"
                f" --port-offset={port_offset}"
                + (" --terminal" if "veh" in self.terminals else "")
                + self.visualizer.add_vehicle_cmd(veh)
            ),
            "logic_cmd": (
                f"python3 -m simulator.logic"
                f' --config-path "{logic_config_path}"'
                f" --verbose {self.verbose}"
            ),
            "socat_cmd": (
                f"socat -d -d"
                f" pty,raw,echo=0,link=/tmp/adsb_{sysid}_ardupilot"
                f" pty,raw,echo=0,link=/tmp/adsb_{sysid}_injector"
            ),
            "adsb_cmd": (
                f"python3 -m simulator.adsb_injector"
                f" --sysid {sysid}"
                f" --port-offset {port_offset}"
                f" --verbose {self.verbose}"
            ),
        }
        return uav_config
