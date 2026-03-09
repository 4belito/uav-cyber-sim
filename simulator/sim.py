"""
Launches multi-UAV simulation with ArduPilot SITL, logic, proxies,
and optional visualization.
"""

import json
import logging
import socket
from pathlib import Path
from typing import Callable, Generic, Literal

from simulator.config import (
    ARDU_LOGS_PATH,
    ARDUPILOT_VEHICLE_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    VEH_PARAMS_PATH,
    BasePort,
)
from simulator.entities import SimGCS, SimVehicle, VehT
from simulator.helpers.processes import create_process
from simulator.helpers.setup_log import setup_logging
from simulator.oracle import Oracle
from simulator.visualizer import Visualizer

SimProcess = Literal[
    "launcher", "veh", "logic", "proxy", "gcs", "adsb_socat", "adsb_injector"
]


class Simulator(Generic[VehT]):
    """
    Manages a full multi-UAV simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"

    def __init__(
        self,
        # visualization
        visualizer: Visualizer[VehT],
        terminals: list[SimProcess] = [],
        supress_output: list[SimProcess] = ["launcher", "adsb_socat", "adsb_injector"],
        verbose: int = 1,
        # oracle
        transmission_range: int = 100,  # meters for inter-UAV communication
    ):
        self.visualizer = visualizer
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = set(terminals)
        self.suppress = set(supress_output)
        self.vehs: dict[int, SimVehicle] = {}
        self.gcs: dict[str, SimGCS] = {}
        self.verbose = verbose
        # self.uav_port_offsets: dict[int, int] = {}
        # self.gcs_port_offsets: dict[str, int] = {}

        self.logic_cmd: Callable[[int, str, int], str] = (
            lambda _, config_path, verbose: (
                f'python3 -m simulator.logic --config-path "{config_path}" '
                f"--verbose {verbose} "
            )
        )
        self.gcs_cmd: Callable[[str, str, int], str] = lambda _, config_path, verbose: (
            f'python3 -m simulator.gcs --config-path "{config_path}" '
            f"--verbose {verbose}"
        )
        self.transmission_range = transmission_range  # meters

        setup_logging(self.oracle_name, verbose=verbose, console_output=True)

    def add_vehicle(self, vehicle: SimVehicle):
        """Add a vehicle to the simulation."""
        self.vehs[vehicle.sysid] = vehicle
        if vehicle.gcs_name not in self.gcs:
            self.gcs[vehicle.gcs_name] = SimGCS(name=vehicle.gcs_name)
        self.gcs[vehicle.gcs_name].sysids.append(vehicle.sysid)
        self.visualizer.add_vehicle(vehicle)

    def launch(self) -> Oracle:
        """Launch vehicle instances and visualizer."""
        uav_port_offsets = self._find_uav_port_offsets()
        gcs_port_offsets = self._find_gcs_port_offsets()
        for veh, offset in zip(self.vehs.values(), uav_port_offsets):
            veh.port_offset = offset
        for gcs, offset in zip(self.gcs.values(), gcs_port_offsets):
            gcs.port_offset = offset
        # self.uav_port_offsets = dict(zip(self.vehs, uav_port_offsets))
        # self.gcs_port_offsets = dict(zip(self.gcs, gcs_port_offsets))
        self._save_logic_configs(DATA_PATH)
        self._save_gcs_configs(DATA_PATH)
        if not self.visualizer.delay:
            self.visualizer.launch(list(uav_port_offsets))
        self._launch_gcses()
        if self.visualizer.delay:
            self.visualizer.launch(list(uav_port_offsets))
        return Oracle(
            self.gra_origin,
            self.vehs,
            self.gcs,
            transmission_range=self.transmission_range,
        )

    def show(self):
        """
        Render a static preview of the configured simulation
        before launch.
        """
        self.visualizer.show()

    def _launch_gcses(self):
        """Launch each GCS process and create an Oracle instance."""
        for gcs_name in self.gcs:
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
        inst = 0
        for gcs_name, gcs in self.gcs.items():
            uavs: list[dict[str, int | str]] = []
            for sysid in gcs.sysids:
                port_offset = self.vehs[sysid].port_offset
                assert port_offset is not None, f"Port offset for UAV {sysid} not set"
                uavs.append(
                    {
                        "sysid": sysid,
                        "port_offset": port_offset,
                        "ardupilot_cmd": (
                            f"python3 {ARDUPILOT_VEHICLE_PATH}"
                            f" -v ArduCopter -I{inst} --sysid {sysid} --no-rebuild"
                            f' -A "--serial5=uart:/tmp/adsb_{sysid}_ardupilot:57600"'
                            f" --use-dir={ARDU_LOGS_PATH}"
                            f" --add-param-file {VEH_PARAMS_PATH}"
                            f" --no-mavproxy"
                            f" --port-offset={port_offset}"
                            + (" --terminal" if "veh" in self.terminals else "")
                            + self.visualizer.add_vehicle_cmd(sysid)
                        ),
                        "logic_cmd": self.logic_cmd(
                            sysid,
                            str(DATA_PATH / f"logic_config_{sysid}.json"),
                            self.verbose,
                        ),
                        # "proxy_cmd": (
                        #     f"python3 -m simulator.proxy --sysid {sysid} "
                        #     f"--port-offset={port_offset} "
                        #     f"--verbose {self.verbose}"
                        # ),
                    }
                )
                inst += 1

            gcs_config = {
                "name": gcs_name,
                "port_offset": gcs.port_offset,
                "uavs": uavs,
                "terminals": list(self.terminals),
                "suppress": list(self.suppress),
            }

            config_path = folder_name / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    # TODO: Check why BasePort.GCS is in find_uav_port_offset
    # and no in find_gcs_port_offset
    def _find_uav_port_offsets(self):
        base_ports = [
            BasePort.ARP,
            BasePort.ARP2,
            BasePort.ARP3,
            BasePort.GCS,
            BasePort.QGC,
            BasePort.LOG,
            BasePort.RID_UP,
            BasePort.RID_DOWN,
            BasePort.RID_DATA,
        ]
        return self._find_port_offsets(base_ports, len(self.vehs))

    # excluded_offsets=[160]

    def _find_gcs_port_offsets(self) -> list[int]:
        base_ports = [BasePort.GCS_ZMQ]
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
