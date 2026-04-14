"""
Launches multi-Vehicle simulation with ArduPilot SITL, logic, and optional
visualization.
"""

import json
import logging
import socket
from typing import Generic

from simulator.config import (
    DATA_PATH,
    ENV_CMD_PYT,
    LOGS_PATH,
    VEH_PARAMS_PATH,
    BasePort,
)
from simulator.configs.gcs import VehicleConfig
from simulator.entities import SimGCS, SimVehicle, VehT
from simulator.external.sitl import ensure_sitl_built, get_default_params
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.helpers.processes import SimProcess, create_process
from simulator.oracle import Oracle
from simulator.params.simulation import SIM_SPEEDUP
from simulator.visualizer import Visualizer

# TODO: remove hard-coded ArduCopter and add it to SimVehicle as firmware


class Simulator(Generic[VehT]):
    """
    Manages a full multi-Vehicle simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"
    logic_folder = DATA_PATH / "logic"
    gcs_folder = DATA_PATH / "gcs"
    port_step = 10

    def __init__(
        self,
        visualizer: Visualizer[VehT],
        terminals: list[SimProcess] = [],
        suppress_output: list[SimProcess] = [
            SimProcess.ARDUPILOT,
            SimProcess.ADSB_SOCAT,
        ],
        verbose: int = 1,
        transmission_range: int = 100,  # meters for inter-Vehicle communication
    ):
        self.visualizer = visualizer
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = set(terminals)
        self.suppress = set(suppress_output)
        self.vehicles: dict[int, SimVehicle] = {}
        self.gcs: dict[str, SimGCS] = {}
        self.orc_port_offset: int | None = None
        self.verbose = verbose
        self.n_instances = 0
        self.parms: dict[int, str] = {}
        # TODO: This is actually cell size and is more an oracle property(check design)
        self.transmission_range = transmission_range  # meters
        setup_logging(
            LOGS_PATH / f"{self.oracle_name}.log", verbose=verbose, console_output=True
        )

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        port_offsets = self._find_port_offsets(
            [
                BasePort.ARP,
                BasePort.ADSB,
                BasePort.ARP2,
                BasePort.ARP3,
                BasePort.RID_UP,
                BasePort.RID_DOWN,
                BasePort.GCS,
            ],
            len(self.vehicles),
        )
        self.orc_port_offset = self._find_port_offsets([BasePort.ORC_DONE], 1)[0]
        port_offsets_dict: dict[int, int] = {}
        for sysid, offset in zip(sorted(self.vehicles), port_offsets, strict=True):
            port_offsets_dict[sysid] = offset
            self.vehicles[sysid].port_offset = offset
        self._save_logic_configs()
        self._save_gcs_configs()
        self.visualizer.launch(port_offsets_dict)
        self._launch_gcses()
        return Oracle(
            self.gra_origin,
            self.vehicles,
            self.gcs,
            transmission_range=self.transmission_range,
            port_offset=self.orc_port_offset,
        )

    def add_vehicle(self, vehicle: SimVehicle, parm: str = str(VEH_PARAMS_PATH)):
        """Add a vehicle to the simulation."""
        self.vehicles[vehicle.sysid] = vehicle
        if vehicle.gcs_name not in self.gcs:
            self.gcs[vehicle.gcs_name] = SimGCS(name=vehicle.gcs_name)
        self.vehicles[vehicle.sysid].instance = self.n_instances
        self.n_instances += 1
        self.gcs[vehicle.gcs_name].sysids.append(vehicle.sysid)
        self.visualizer.add_vehicle(vehicle)

        # DEBUG
        self.parms[vehicle.sysid] = parm

    def remove_vehicle(self, sysid: int) -> bool:
        """Remove a vehicle by system ID."""
        if sysid in self.vehicles:
            del self.vehicles[sysid]
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
            gcs_config_path = self.gcs_folder / f"gcs_config_{gcs_name}.json"
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

    def _save_logic_configs(self):
        """Save the logic configurations for each Vehicle."""
        self.logic_folder.mkdir(parents=True, exist_ok=True)
        for sysid, veh in self.vehicles.items():
            logic_config = {
                "sysid": sysid,
                "gra_origin_dict": {
                    "lat": self.gra_origin.lat,
                    "lon": self.gra_origin.lon,
                    "alt": self.gra_origin.alt,
                },
                "veh_port_offset": veh.port_offset,
                "oracle_port_offset": self.orc_port_offset,
                "plan_spec": veh.plan.get_spec().to_dict(),
            }
            config_path = self.logic_folder / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self):
        self.gcs_folder.mkdir(parents=True, exist_ok=True)
        for gcs_name, gcs in self.gcs.items():
            gcs_config = {
                "name": gcs_name,
                "oracle_port_offset": self.orc_port_offset,
                "vehicles": [self._build_veh_config(sysid) for sysid in gcs.sysids],
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }

            config_path = self.gcs_folder / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _find_port_offsets(
        self,
        base_ports: list[BasePort],
        n_ports: int,
    ) -> list[int]:
        """Find available port offsets for each Vehicle to avoid conflicts."""
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
            cur_offset += self.port_step
        return offsets

    def _build_veh_config(self, sysid: int) -> VehicleConfig:
        veh = self.vehicles[sysid]

        port_offset = veh.port_offset_required
        inst = port_offset // self.port_step
        binary = ensure_sitl_built(
            frame=veh.model,  # e.g. "gazebo-iris"
            firmware=veh.firmware,  # or "ArduPlane", etc.
        )
        veh_parms = self.parms[veh.sysid]
        arp_cmd = [
            str(binary),
            "--model",
            veh.model,
            "-I" + str(inst),
            "--speedup",
            str(SIM_SPEEDUP),
            "--sysid",
            str(connection_id(sysid)),
            "--base-port",
            str(BasePort.ARP + port_offset),
            "--slave 0",
            "--sim-address=127.0.0.1",
            "--home",
            self.visualizer.home_str(veh),
            f"--serial5=uart:/tmp/adsb_{sysid}_ardupilot:57600",
            "--defaults",
            ",".join(get_default_params(veh.model, veh.firmware) + [veh_parms]),
        ]

        arp_cmd.extend(self.visualizer.add_sitl_args(veh))
        logic_config_path = str(self.logic_folder / f"logic_config_{sysid}.json")
        veh_config: VehicleConfig = {
            "sysid": sysid,
            "veh_port_offset": port_offset,
            "ardupilot_cmd": " ".join(arp_cmd),
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
        return veh_config
