"""
Launches multi-Vehicle simulation with ArduPilot SITL, logic, and optional
visualization.
"""

import json
import logging
import socket
from collections.abc import Container, Iterable, Sequence
from subprocess import Popen
from typing import ClassVar, Generic

from simulator.config import (
    ARDU_LOGS_PATH,
    DATA_PATH,
    ENV_CMD_PYT,
    GCS_TELEM_WINDOW,
    LOGS_PATH,
    SITL_INSTANCE_STRIDE,
    SimPort,
    SitlPort,
    VehPort,
)
from simulator.configs.gcs import VehicleConfig
from simulator.entities import VehT
from simulator.external.sitl import resolve_sitl_build
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.helpers.processes import SimProcess, create_process
from simulator.oracle import Oracle
from simulator.runtime.vehicle_launcher import launch_vehicle
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
    # Every vehicle claims `base + its offset` for each of these, plus the whole
    # GCS telemetry window — so a vehicle can be monitored by that many GCSs
    # before it runs out of listeners. Claimed as one atomic block.
    veh_base_ports: ClassVar[list[int]] = [
        VehPort.ARP,
        VehPort.ADSB,
        VehPort.ARP2,
        VehPort.ARP3,
        VehPort.RID_UP,
        VehPort.RID_DOWN,
        VehPort.GCS_CMD,
        VehPort.MITM_TELEM,
        VehPort.MITM_CMD,
        # The window is SITL_INSTANCE_STRIDE ports per vehicle, but the allocator
        # understands one port per base — expand it into that many entries so
        # the whole window is claimed with the block.
        *range(GCS_TELEM_WINDOW, GCS_TELEM_WINDOW + SITL_INSTANCE_STRIDE),
        # Claimed by SITL itself at the same stride; nothing of ours binds them,
        # but the block is not really free unless they are.
        *SitlPort,
    ]
    max_gcss_per_veh: ClassVar[int] = SITL_INSTANCE_STRIDE

    def __init__(
        self,
        visualizer: Visualizer[VehT],
        oracle: Oracle,
        terminals: list[SimProcess] | None = None,
        suppress_output: list[SimProcess] | None = None,
        verbose: int = 1,
        speedup: float = 1.0,
    ):
        if terminals is None:
            terminals = []
        if suppress_output is None:
            suppress_output = [SimProcess.ARDUPILOT, SimProcess.ADSB_SOCAT]
        self.visualizer = visualizer
        # The Oracle holds the scenario (vehicles, GCSs, interventions, MITM);
        # the Simulator only decides how to execute it. `launch()` binds it.
        self.oracle = oracle
        self.gra_origin = self.visualizer.gra_origin
        self.terminals = terminals
        self.suppress = suppress_output
        # Telemetry listener port per (vehicle, GCS), aligned with `veh.gcss`.
        self.veh_telem_ports: dict[int, list[int]] = {}
        # Processes of the vehicles no GCS monitors, which the Simulator owns.
        self.unassigned_procs: dict[int, dict[SimProcess, Popen[bytes]]] = {}
        self.orc_port_offset: int
        self.verbose = verbose
        # SITL wall-clock multiplier: how fast the run goes, not what it runs.
        self.speedup = speedup
        setup_logging(
            LOGS_PATH / f"{self.oracle_name}.log", verbose=verbose, console_output=True
        )

    def launch(self) -> None:
        """
        Launch the vehicles, the visualizer and the GCSs.

        Binds `self.oracle` to the run as its last step, so `simulator.oracle`
        (or the Oracle you passed in) is ready to `run()` when this returns.
        """
        # Simulation-wide ports are claimed first: there are only a couple and
        # QGC's is fixed by QGroundControl, whereas the vehicle blocks are many
        # and free to move. The vehicle search then excludes them, so the two
        # groups stay disjoint however far the blocks climb (a vehicle's
        # MITM_TELEM would otherwise reach QGC's 14550 at ~556 vehicles).
        self.orc_port_offset = self._find_port_offsets(
            [SimPort.ORC_DONE], 1, {int(SimPort.QGC)}
        )[0]
        sim_ports = {int(SimPort.QGC), SimPort.ORC_DONE + self.orc_port_offset}
        self._sync_visualizer()
        port_offsets = self._find_port_offsets(
            self.veh_base_ports, len(self.oracle.vehicles), sim_ports
        )
        port_offsets_dict: dict[int, int] = {}
        for sysid, offset in zip(
            sorted(self.oracle.vehicles), port_offsets, strict=True
        ):
            port_offsets_dict[sysid] = offset
            self.oracle.vehicles[sysid].port_offset = offset
        self._assign_telem_ports()
        self._save_logic_configs()
        self._save_gcs_configs()
        self.visualizer.launch(port_offsets_dict)
        self._launch_gcses()
        self._launch_unassigned_vehicles()
        # Everything the Oracle needs exists only now, so it is wired up here
        # rather than by the caller.
        self.oracle.bind(self.gra_origin, port_offset=self.orc_port_offset)

    def preview(self):
        """
        Render a static preview of the configured simulation
        before launch.
        """
        self._sync_visualizer()
        self.visualizer.preview()

    def _sync_visualizer(self) -> None:
        """
        Register the scenario's vehicles with the visualizer.

        Done here rather than when a vehicle is added, so the Oracle can own the
        scenario without needing to know about the visualizer, and vehicles may
        be added before or after the Simulator is built.
        """
        for veh in self.oracle.vehicles.values():
            self.visualizer.add_vehicle(veh)

    def _launch_gcses(self):
        """Launch each GCS process and create an Oracle instance."""
        for gcs_name, gcs in self.oracle.gcss.items():
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

    def run(self) -> None:
        """
        Launch the simulation and block until every mission completes.

        Convenience for the usual `launch()` then `oracle.run()` pairing. Call
        the two separately when you need to do something in between — inspect
        the spawned processes, or watch the visualizer come up before flying.
        """
        self.launch()
        self.oracle.run()

    def _launch_unassigned_vehicles(self) -> None:
        """
        Launch the vehicles no GCS monitors.

        A vehicle's processes are normally spawned by its first GCS; one with no
        GCS at all still has to fly, so the Simulator owns it instead. It emits
        no telemetry (`gcs_telem_ports` is empty) and reports completion to the
        Oracle directly from its logic process.
        """
        for sysid, veh in self.oracle.vehicles.items():
            if veh.gcss:
                continue
            veh_config = self._build_veh_config(
                sysid,
                telem_port=GCS_TELEM_WINDOW + veh.port_offset_required,
                launch=True,
            )
            self.unassigned_procs[sysid] = launch_vehicle(
                veh_config, self.terminals, self.suppress
            )
            logging.info(f"🚀 Vehicle {sysid} launched (monitored by no GCS)")

    def _assign_telem_ports(self) -> None:
        """
        Assign one telemetry listener port per (vehicle, GCS) pair.

        A UDP port has a single binder, so each GCS watching a vehicle needs its
        own: the k-th takes `GCS_TELEM_WINDOW + k + offset` from the window
        already claimed with the rest of the vehicle's block. Every port is thus
        derivable from the vehicle's offset, and the list is aligned
        index-by-index with `veh.gcss`.
        """
        for sysid, veh in self.oracle.vehicles.items():
            offset = veh.port_offset_required
            if len(veh.gcss) > self.max_gcss_per_veh:
                raise ValueError(
                    f"Vehicle {sysid} is monitored by {len(veh.gcss)} GCSs, but "
                    f"only {self.max_gcss_per_veh} telemetry ports fit in its "
                    f"window. The ceiling is ArduPilot's instance stride, which "
                    f"cannot be raised — a second window base "
                    f"would be needed instead."
                )
            self.veh_telem_ports[sysid] = [
                GCS_TELEM_WINDOW + k + offset for k in range(len(veh.gcss))
            ]

    def _save_logic_configs(self):
        """Save the logic configurations for each Vehicle."""
        self.logic_dir.mkdir(parents=True, exist_ok=True)
        for sysid, veh in self.oracle.vehicles.items():
            home_heading = self.visualizer.gra_home(veh).heading
            logic_config = {
                "sysid": sysid,
                "gra_origin_dict": self.gra_origin.unpose()._asdict(),
                "home_heading": home_heading,
                "veh_port_offset": veh.port_offset,
                "oracle_port_offset": self.orc_port_offset,
                "plan_spec": veh.plan.get_spec().to_dict(),
                "mitm": sysid in self.oracle.mitm,
                "gcs_telem_ports": self.veh_telem_ports[sysid],
                "rid_frequency": self.oracle.rid_frequency,
            }
            config_path = self.logic_dir / f"logic_config_{sysid}.json"
            with config_path.open("w") as f:
                json.dump(logic_config, f, indent=2)

    def _save_gcs_configs(self):
        self.gcs_dir.mkdir(parents=True, exist_ok=True)
        for gcs_name, gcs in self.oracle.gcss.items():
            terminals = self.terminals if gcs.terminals is None else gcs.terminals
            suppress = self.suppress if gcs.suppress is None else gcs.suppress
            veh_configs: list[VehicleConfig] = []
            for veh in gcs.vehicles:
                # Where this GCS sits in the vehicle's list decides both its
                # telemetry port and whether it owns the vehicle's processes.
                idx = next(i for i, g in enumerate(veh.gcss) if g is gcs)
                veh_configs.append(
                    self._build_veh_config(
                        veh.sysid,
                        telem_port=self.veh_telem_ports[veh.sysid][idx],
                        launch=idx == 0,
                    )
                )
            gcs_config = {
                "name": gcs_name,
                "oracle_port_offset": self.orc_port_offset,
                "record_positions": gcs.record_positions,
                "vehicles": veh_configs,
                "terminals": terminals,
                "suppress": suppress,
            }

            config_path = self.gcs_dir / f"gcs_config_{gcs_name}.json"
            with config_path.open("w") as f:
                json.dump(gcs_config, f, indent=2)

    def _ports_available(self, ports: Iterable[int], reserved: Container[int]) -> bool:
        """
        Whether every one of `ports` is free to bind and unclaimed.

        Both transports are probed. A block mixes TCP users (SITL serials, the
        ZMQ sockets) with UDP ones (the GCS links, SITL's own RCIN/Gazebo
        ports), and testing only one would hand out a port another process
        already holds on the other.

        `reserved` names ports already spoken for but not yet bound, which the
        probe alone would wrongly report as free.
        """
        for port in ports:
            if port in reserved:
                return False
            for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
                with socket.socket(socket.AF_INET, kind) as sock:
                    sock.settimeout(0.01)
                    try:
                        sock.bind(("0.0.0.0", port))
                    except Exception:
                        return False
        return True

    def _find_port_offsets(
        self,
        base_ports: Sequence[int],
        n_ports: int,
        reserved: Container[int] = frozenset(),
    ) -> list[int]:
        """
        Find `n_ports` offsets whose whole per-vehicle block is available.

        An offset is taken only if *every* base port is free at it, so a
        vehicle's ports always come as one stride-wide block.
        """
        offsets: list[int] = []
        offset = 0
        while len(offsets) < n_ports:
            if self._ports_available((b + offset for b in base_ports), reserved):
                offsets.append(offset)
            offset += SITL_INSTANCE_STRIDE
        return offsets

    def _build_veh_config(
        self, sysid: int, telem_port: int, launch: bool
    ) -> VehicleConfig:
        veh = self.oracle.vehicles[sysid]

        port_offset = veh.port_offset_required
        eeprom_path = ARDU_LOGS_PATH / f"veh_{sysid}" / "eeprom.bin"
        if eeprom_path.exists():
            eeprom_path.unlink()
        inst = port_offset // SITL_INSTANCE_STRIDE
        frame = veh.model(self.visualizer.name)
        firmware = veh.model.firmware
        binary, sitl_model, default_params = resolve_sitl_build(frame, firmware)
        arp_cmd = [
            str(binary),
            "--model",
            sitl_model,
            "-I" + str(inst),
            "--speedup",
            str(self.speedup),
            "--sysid",
            str(connection_id(sysid)),
            "--base-port",
            str(VehPort.ARP + port_offset),
            "--slave 0",
            "--sim-address=127.0.0.1",
            "--home",
            self.visualizer.gra_home(veh).to_str(),
            f"--serial5=uart:/tmp/adsb_{sysid}_ardupilot:57600",
            "--defaults",
            ",".join(default_params + [veh.parm]),
        ]

        arp_cmd.extend(self.visualizer.add_sitl_args(veh))
        logic_config_path = str(self.logic_dir / f"logic_config_{sysid}.json")
        mitm_config = self.oracle.mitm.get(sysid)
        mitm_enabled = mitm_config is not None
        strategy = mitm_config["strategy"] if mitm_config is not None else "passthrough"
        mitm_params = mitm_config.get("params", {}) if mitm_config is not None else {}
        # The MITM sits between Logic and the GCSs, so it is what fans the
        # telemetry out; it needs every monitoring GCS's port, not just this one.
        # An unmonitored vehicle has none, and the flag is then left off.
        telem_ports = ",".join(str(port) for port in self.veh_telem_ports[sysid])
        telem_ports_arg = f" --telem-ports {telem_ports}" if telem_ports else ""
        mitm_cmd = (
            (
                f"python3 -m simulator.mitm"
                f" --sysid {sysid}"
                f" --port-offset {port_offset}"
                f" --strategy {strategy}"
                f" --params '{json.dumps(mitm_params)}'"
                f"{telem_ports_arg}"
                f" --verbose {self.verbose}"
            )
            if mitm_enabled
            else ""
        )
        veh_config: VehicleConfig = {
            "sysid": sysid,
            "veh_port_offset": port_offset,
            "telem_port": telem_port,
            "launch": launch,
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
            "intervention": self.oracle.intervention.get(sysid),
        }
        return veh_config
