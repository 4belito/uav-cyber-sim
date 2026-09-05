"""
Launches multi-Vehicle simulation with ArduPilot SITL, logic, and optional
visualization.
"""

import json
import logging
import socket
from typing import Generic

from simulator.config import (
    ARDU_LOGS_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    LOGS_PATH,
    VEH_PARAMS_PATH,
    BasePort,
)
from simulator.configs.gcs import VehicleConfig
from simulator.configs.mitm import MITMConfig
from simulator.entities import SimGCS, SimVehicle, VehT
from simulator.external.sitl import resolve_sitl_build
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.helpers.processes import SimProcess, create_process
from simulator.params.simulation import SIM_SPEEDUP
from simulator.visualizer import Visualizer

# TODO: remove hard-coded ArduCopter and add it to SimVehicle as firmware


class Simulator(Generic[VehT]):
    """
    Manages a full multi-Vehicle simulation, including SITL, logic, proxy, GCS,
    and visualization.
    """

    oracle_name: str = "Oracle ⚪"
    logic_dir = DATA_PATH / "logic"
    gcs_dir = DATA_PATH / "gcs"
    port_step = 10

    def __init__(
        self,
        visualizer: Visualizer[VehT],
        terminals: list[SimProcess] | None = None,
        suppress_output: list[SimProcess] | None = None,
        verbose: int = 1,
    ):
        if terminals is None:
            terminals = []
        if suppress_output is None:
            suppress_output = [SimProcess.ARDUPILOT, SimProcess.ADSB_SOCAT]
        self.visualizer = visualizer
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = terminals
        self.suppress = suppress_output
        self.vehicles: dict[int, SimVehicle] = {}
        self.gcs: dict[str, SimGCS] = {}
        self.orc_port_offset: int
        self.verbose = verbose
        self.n_instances = 0
        self.veh_parms: dict[int, str] = {}
        self.intervention: dict[int, dict[str, float]] = {}
        self.mitm: dict[int, MITMConfig] = {}
        setup_logging(
            LOGS_PATH / f"{self.oracle_name}.log", verbose=verbose, console_output=True
        )

    def launch(self) -> None:
        """Launch vehicle instances and visualizer."""
        # After this returns, gra_origin/vehicles/gcs/orc_port_offset are ready
        # to construct an Oracle for the run.
        port_offsets = self._find_port_offsets(
            [
                BasePort.ARP,
                BasePort.ADSB,
                BasePort.ARP2,
                BasePort.ARP3,
                BasePort.RID_UP,
                BasePort.RID_DOWN,
                BasePort.GCS,
                BasePort.GCS_CMD,
                BasePort.MITM_TELEM,
                BasePort.MITM_CMD,
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

    def add_vehicle(self, vehicle: SimVehicle, parm: str = str(VEH_PARAMS_PATH)):
        """Add a vehicle to the simulation."""
        self.vehicles[vehicle.sysid] = vehicle
        registered_gcs = self.gcs.get(vehicle.gcs.name)
        if registered_gcs is None:
            self.gcs[vehicle.gcs.name] = vehicle.gcs
        elif registered_gcs is not vehicle.gcs:
            raise ValueError(
                f"A different GCS instance named '{vehicle.gcs.name}' is "
                "already registered; share one SimGCS across its vehicles."
            )
        self.vehicles[vehicle.sysid].instance = self.n_instances
        self.n_instances += 1
        vehicle.gcs.sysids.append(vehicle.sysid)
        self.visualizer.add_vehicle(vehicle)
        self.veh_parms[vehicle.sysid] = parm

    def preview(self):
        """
        Render a static preview of the configured simulation
        before launch.
        """
        self.visualizer.preview()

    def _launch_gcses(self):
        """Launch each GCS process and create an Oracle instance."""
        for gcs_name, gcs in self.gcs.items():
            terminals = self.terminals if gcs.terminals is None else gcs.terminals
            suppress = self.suppress if gcs.suppress is None else gcs.suppress
            verbose = self.verbose if gcs.verbose is None else gcs.verbose
            gcs_config_path = self.gcs_dir / f"gcs_config_{gcs_name}.json"
            gcs_cmd = (
                f'python3 -m simulator.gcs --config-path "{gcs_config_path}"'
                f" --verbose {verbose}"
            )
            p = create_process(
                gcs_cmd,
                after="exec bash",
                visible=SimProcess.GCS in terminals,
                suppress_output=SimProcess.GCS in suppress,
                title=f"GCS: {gcs_name}",
                env_cmd=ENV_CMD_PYT,
            )  # "exit"
            logging.info(f"🚀 GCS {gcs_name} launched (PID {p.pid})")

    def _save_logic_configs(self):
        """Save the logic configurations for each Vehicle."""
        self.logic_dir.mkdir(parents=True, exist_ok=True)
        for sysid, veh in self.vehicles.items():
            home_heading = self.visualizer.gra_home(veh).heading
            logic_config = {
                "sysid": sysid,
                "gra_origin_dict": self.gra_origin.unpose()._asdict(),
                "home_heading": home_heading,
                "veh_port_offset": veh.port_offset,
                "oracle_port_offset": self.orc_port_offset,
                "plan_spec": veh.plan.get_spec().to_dict(),
                "mitm": sysid in self.mitm,
            }
            config_path = self.logic_dir / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self):
        self.gcs_dir.mkdir(parents=True, exist_ok=True)
        for gcs_name, gcs in self.gcs.items():
            terminals = self.terminals if gcs.terminals is None else gcs.terminals
            suppress = self.suppress if gcs.suppress is None else gcs.suppress
            gcs_config = {
                "name": gcs_name,
                "oracle_port_offset": self.orc_port_offset,
                "vehicles": [self._build_veh_config(sysid) for sysid in gcs.sysids],
                "terminals": terminals,
                "suppress": suppress,
            }

            config_path = self.gcs_dir / f"gcs_config_{gcs_name}.json"
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
        veh_parms = self.veh_parms[veh.sysid]
        eeprom_path = ARDU_LOGS_PATH / f"veh_{sysid}" / "eeprom.bin"
        if eeprom_path.exists():
            eeprom_path.unlink()
        inst = port_offset // self.port_step
        frame = veh.model(self.visualizer.name)
        firmware = veh.model.firmware
        binary, sitl_model, default_params = resolve_sitl_build(frame, firmware)
        arp_cmd = [
            str(binary),
            "--model",
            sitl_model,
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
            self.visualizer.gra_home(veh).to_str(),
            f"--serial5=uart:/tmp/adsb_{sysid}_ardupilot:57600",
            "--defaults",
            ",".join(default_params + [veh_parms]),
        ]

        arp_cmd.extend(self.visualizer.add_sitl_args(veh))
        logic_config_path = str(self.logic_dir / f"logic_config_{sysid}.json")
        mitm_config = self.mitm.get(sysid)
        mitm_enabled = mitm_config is not None
        strategy = mitm_config["strategy"] if mitm_config is not None else "passthrough"
        mitm_params = mitm_config.get("params", {}) if mitm_config is not None else {}
        mitm_cmd = (
            (
                f"python3 -m simulator.mitm"
                f" --sysid {sysid}"
                f" --port-offset {port_offset}"
                f" --strategy {strategy}"
                f" --params '{json.dumps(mitm_params)}'"
                f" --verbose {self.verbose}"
            )
            if mitm_enabled
            else ""
        )
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
            "mitm": mitm_enabled,
            "mitm_cmd": mitm_cmd,
            "intervention": self.intervention.get(sysid),
        }
        return veh_config
