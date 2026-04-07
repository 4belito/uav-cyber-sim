"""
Define the GCS class to monitor Vehicles through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import logging
import os
import pickle
import time
from concurrent import futures
from subprocess import Popen

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

from simulator.config import (
    ARDU_LOGS_PATH,
    DATA_PATH,
    ENV_CMD_ARP,
    ENV_CMD_PYT,
    LOGS_PATH,
    BasePort,
)
from simulator.configs import VehicleConfig
from simulator.helpers.connections import create_udp_conn, create_zmq_socket
from simulator.helpers.connections.mavlink.customenums.customcmd import CustomCmd
from simulator.helpers.coordinates import GRA, GRAs
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.processes import (
    SimProcess,
    create_process,
    terminate_process_group,
)
from simulator.params.simulation import HEARTBEAT_FREQUENCY
from simulator.runtime.gcs_runtime import VehicleRuntime

heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)


def main():
    """Run a GCS instance to monitor Vehicles."""
    config_path, verbose = parse_arguments()
    with open(config_path) as f:
        gcs_config = json.load(f)
    setup_logging(
        LOGS_PATH / "GCSs" / f"GCS_{gcs_config['name']}.log",
        verbose=verbose,
        console_output=True,
    )
    gcs = GCS(**gcs_config)
    gcs.run()


class GCS:
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(
        self,
        name: str,
        vehicles: list[VehicleConfig],
        oracle_port_offset: int,
        terminals: list[SimProcess],
        suppress: list[SimProcess],
    ) -> None:
        # Configure logging for this GCS process
        self.name = name
        self.vehicles = vehicles
        self.sysids = [vehconfig["sysid"] for vehconfig in vehicles]
        self.n_vehicles = len(self.sysids)
        self.terminals = set(terminals)
        self.suppress = set(suppress)
        self.vehruntimes = {vehrun.sysid: vehrun for vehrun in self._launch_vehicles()}
        self.conns = {sysid: vehrun.conn for sysid, vehrun in self.vehruntimes.items()}
        self._ctx = zmq.Context()
        self._done_sock = create_zmq_socket(
            self._ctx,
            zmq.DEALER,
            BasePort.ORC_DONE,
            offset=oracle_port_offset,
            timeout=-1,
            identity=f"gcs-{self.name}".encode(),
        )

        # Data structures for trajectory logging
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.sysids}
        self.pos: dict[int, GRA] = {sysid: GRA.nan() for sysid in self.sysids}
        logging.info(f" GCS {self.name} started with {self.n_vehicles} Vehicles")

    ###
    def run(self):
        """Run the GCS monitoring loop until all Vehicles complete their missions."""
        sysids_snapshot = tuple(self.sysids)

        with futures.ThreadPoolExecutor(max_workers=len(sysids_snapshot)) as executor:
            futures_list = [
                executor.submit(self._monitor_vehicle, sysid)
                for sysid in sysids_snapshot
            ]

            for f in futures.as_completed(futures_list):
                f.result()
        logging.info("All Vehicles assigned have completed their missions")
        self._done_sock.send_string("DONE")  # type: ignore
        logging.info("DONE message sent to Oracle")
        self._wait_until_ack()

        trajectory_file = DATA_PATH / f"trajectories_{self.name}.pkl"
        with open(trajectory_file, "wb") as file:
            pickle.dump(self.paths, file)
        logging.info(f"Trajectories saved to '{trajectory_file}'")

        self._done_sock.close(linger=0)
        self._ctx.term()

    def _wait_until_ack(self):
        """Wait until Oracle acknowledges DONE message."""
        while True:
            try:
                msg = self._done_sock.recv_string()
            except zmq.Again:
                continue

            if msg == "ACK":
                return
            else:
                logging.warning(f"GCS {self.name} ignoring unexpected message: {msg}")

    def _launch_vehicles(self) -> list[VehicleRuntime]:
        """Launch ArduPilot and logic processes for each Vehicle."""
        with futures.ThreadPoolExecutor() as executor:
            vehruns = list(executor.map(self._launch_vehicle, range(self.n_vehicles)))
        return vehruns

    def _launch_vehicle(self, i: int) -> VehicleRuntime:
        veh_config = self.vehicles[i]
        sysid = veh_config["sysid"]

        procs: dict[SimProcess, Popen[bytes]] = {}
        # -----------------------
        # 1. ADS-B virtual cable
        # -----------------------
        p_socat = create_process(
            veh_config["socat_cmd"],
            after="exec bash",
            visible=SimProcess.ADSB_SOCAT in self.terminals,
            suppress_output=SimProcess.ADSB_SOCAT in self.suppress,
            title=f"ADSB socat: Vehicle {sysid}",
            new_process_group=True,
        )
        logging.debug(f"ADSB socat for vehicle {sysid} launched (PID {p_socat.pid})")
        procs[SimProcess.ADSB_SOCAT] = p_socat
        self._wait_for_pty(f"/tmp/adsb_{sysid}_injector")

        # -----------------------
        # 2. ADS-B injector
        # -----------------------

        p_adsb = create_process(
            veh_config["adsb_cmd"],
            after="exec bash",
            visible=SimProcess.ADSB_INJECTOR in self.terminals,
            suppress_output=SimProcess.ADSB_INJECTOR in self.suppress,
            title=f"ADSB injector: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
            new_process_group=True,
        )
        logging.debug(f"ADSB injector for vehicle {sysid} launched (PID {p_adsb.pid})")
        procs[SimProcess.ADSB_INJECTOR] = p_adsb
        # -----------
        # 3. Logic
        # -----------
        p_logic = create_process(
            veh_config["logic_cmd"],
            after="exec bash",
            visible=SimProcess.LOGIC in self.terminals,
            suppress_output=SimProcess.LOGIC in self.suppress,
            title=f"Vehicle logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
            new_process_group=True,
        )  # "exit"
        logging.debug(f"Vehicle logic for vehicle {sysid} launched (PID {p_logic.pid})")
        procs[SimProcess.LOGIC] = p_logic

        # ----------------
        # 3. ArduPilot
        # ----------------
        ardu_log_folder = ARDU_LOGS_PATH / f"veh_{sysid}"
        ardu_log_folder.mkdir(parents=True, exist_ok=True)
        p_ard = create_process(
            veh_config["ardupilot_cmd"],
            after="exec bash",
            visible=SimProcess.ARDUPILOT in self.terminals,
            suppress_output=SimProcess.ARDUPILOT in self.suppress,
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
            new_process_group=True,
            cwd=str(ardu_log_folder),
        )  # "exit"
        logging.debug(f"ArduPilot SITL vehicle {sysid} launched (PID {p_ard.pid})")
        procs[SimProcess.ARDUPILOT] = p_ard

        ## create MAVLink connection to the SITL instance for this Vehicle
        conn = create_udp_conn(
            base_port=BasePort.GCS,
            offset=veh_config["veh_port_offset"],
            mode="receiver",
            src_sysid=255,  # estándar GCS sysid
            src_compid=190,  # estándar GCS commponent ID
        )
        logging.info(f"Vehicle {sysid} connected")
        return VehicleRuntime(sysid=sysid, conn=conn, processes=procs)

    def _monitor_vehicle(self, sysid: int):
        logging.info(f"Monitoring Vehicle {sysid}")
        try:
            while not self._is_vehicle_plan_done(sysid):
                # self._get_global_pos(sysid)
                # self._save_pos()
                pass
        finally:
            self._remove_vehicle(sysid)
            logging.debug(f"Monitor thread finished for Vehicle {sysid}")

    def _save_pos(self):
        """Save the current global position of each Vehicle to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)

    def _remove_vehicle(self, sysid: int):
        """Remove vehicles from the environment."""
        self.conns[sysid].close()
        del self.conns[sysid]
        del self.vehruntimes[sysid]
        del self.sysids[self.sysids.index(sysid)]
        self._terminate_veh_processes(sysid)
        self.n_vehicles -= 1
        logging.info(f"Vehicle {sysid} removed from GCS {self.name}")

    # @staticmethod
    # def load_config(config_path: str) -> GCSConfig:
    #     """Load GCS configuration from a JSON file via command line argument."""
    #     with open(config_path) as f:
    #         gcs_config: GCSConfig = json.load(f)
    #     return gcs_config

    def _wait_for_pty(self, path: str, timeout: float = 3.0):
        t0 = time.time()
        while not os.path.exists(path):
            if time.time() - t0 > timeout:
                raise RuntimeError(f"PTY not created: {path}")
            time.sleep(0.05)

    def _is_vehicle_plan_done(self, sysid: int) -> bool:
        """
        Listen for a STATUSTEXT('LOGIC_DONE') message and respond with a
        COMMAND_ACK mavlink message.
        """
        conn = self.conns[sysid]
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)

        if not msg:
            return False

        if msg.text == "LOGIC_DONE":
            conn.mav.command_ack_send(
                command=CustomCmd.LOGIC_DONE,
                result=mavlink.MAV_RESULT_ACCEPTED,
            )
            logging.info(f"✅ Vehicle {sysid} completed its mission")
            return True

        return False

    def _terminate_veh_processes(self, sysid: int) -> None:
        runtime = self.vehruntimes.get(sysid)
        if runtime is None:
            logging.debug(f"No runtime found for Vehicle {sysid}")
            return

        for name, proc in runtime.processes.items():
            terminate_process_group(proc, f"{name} for Vehicle {sysid}")

    def _get_global_pos(self, sysid: int):
        """Get the current global position of the specified vehicle."""
        msg = self.conns[sysid].recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=0.001
        )

        if not msg:
            return None

        self.pos[sysid] = GRA.from_global_int(msg.lat, msg.lon, msg.relative_alt)


def parse_arguments() -> tuple[str, int]:
    """Parse List of GCS system IDs and GCS name."""
    parser = argparse.ArgumentParser(description="Single GCS")
    parser.add_argument(
        "--config-path",
        type=str,
        required=True,
        help="Path to the GCS configuration file (e.g. gcs_config_1.json)",
    )
    parser.add_argument(
        "--verbose",
        type=int,
        required=False,
        default=1,
        help="Verbosity level (0=silent, 1=normal, 2=debug)",
    )
    args = parser.parse_args()
    return args.config_path, args.verbose


if __name__ == "__main__":
    main()
