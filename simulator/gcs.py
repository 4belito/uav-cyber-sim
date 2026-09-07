"""
Define the GCS class to monitor Vehicles through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import logging
import pickle
import time
from concurrent import futures
from subprocess import Popen

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

from simulator.config import (
    DATA_PATH,
    LOGS_PATH,
    SimPort,
    VehPort,
)
from simulator.configs import VehicleConfig
from simulator.helpers.connections import create_udp_conn, create_zmq_socket
from simulator.helpers.connections.mavlink.customenums.customcmd import CustomCmd
from simulator.helpers.connections.mavlink.streams import make_json_safe
from simulator.helpers.coordinates import GRA, GRAs
from simulator.helpers.logging.data_logger import DataLogger
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.processes import SimProcess, terminate_process_group
from simulator.params.simulation import HEARTBEAT_FREQUENCY
from simulator.runtime.gcs_runtime import VehicleRuntime
from simulator.runtime.vehicle_launcher import launch_vehicle

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
    """Ground Control Station for monitoring vehicles and logging trajectories."""

    def __init__(
        self,
        name: str,
        vehicles: list[VehicleConfig],
        oracle_port_offset: int,
        terminals: list[SimProcess],
        suppress: list[SimProcess],
        record_positions: bool = True,
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
            SimPort.ORC_DONE,
            offset=oracle_port_offset,
            timeout=-1,
            identity=f"gcs-{self.name}".encode(),
        )

        self.interventions: dict[int, dict[str, float] | None] = {
            vehconfig["sysid"]: vehconfig.get("intervention") for vehconfig in vehicles
        }

        # Trajectory logging: filled straight from the monitor loop's own
        # GLOBAL_POSITION_INT messages, so nothing competes for the socket.
        self.record_positions = record_positions
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.sysids}
        logging.info(f" GCS {self.name} started with {self.n_vehicles} Vehicles")

    ###
    def run(self):
        """Run the GCS monitoring loop until all Vehicles complete their missions."""
        sysids_snapshot = tuple(self.sysids)

        # A GCS may legitimately monitor no vehicle at all; it then has nothing
        # to wait for and reports DONE straight away.
        if sysids_snapshot:
            with futures.ThreadPoolExecutor(
                max_workers=len(sysids_snapshot)
            ) as executor:
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

        if self.record_positions:
            trajectory_file = DATA_PATH / f"trajectories_{self.name}.pkl"
            with open(trajectory_file, "wb") as file:
                pickle.dump(self.paths, file)
            n = sum(len(p) for p in self.paths.values())
            logging.info(f"{n} positions saved to '{trajectory_file}'")

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
        """Bring up one monitored vehicle and open its telemetry/command links."""
        veh_config = self.vehicles[i]
        sysid = veh_config["sysid"]

        # Only the vehicle's owning GCS spawns its processes; the other GCSs
        # monitoring the same vehicle attach to the already-running one.
        procs: dict[SimProcess, Popen[bytes]] = {}
        if veh_config["launch"]:
            procs = launch_vehicle(veh_config, self.terminals, self.suppress)
        else:
            logging.debug(
                f"Vehicle {sysid} processes owned by another GCS; monitoring only"
            )

        ## create MAVLink connection to the SITL instance for this Vehicle
        # `telem_port` is this GCS's own slot in the vehicle's telemetry window;
        # every GCS watching the vehicle gets a distinct one.
        conn = create_udp_conn(
            base_port=veh_config["telem_port"],
            offset=0,
            mode="receiver",
            src_sysid=255,  # estándar GCS sysid
            src_compid=190,  # estándar GCS commponent ID
        )
        # When a MITM is interposed, send commands to its listener instead of
        # directly to Logic; the MITM relays them onward to VehPort.GCS_CMD.
        cmd_base = VehPort.MITM_CMD if veh_config["mitm"] else VehPort.GCS_CMD
        cmd_conn = create_udp_conn(
            base_port=cmd_base,
            offset=veh_config["veh_port_offset"],
            mode="sender",
            src_sysid=255,
            src_compid=190,
        )
        logging.info(f"Vehicle {sysid} connected")
        return VehicleRuntime(
            sysid=sysid, conn=conn, cmd_conn=cmd_conn, processes=procs
        )

    def _monitor_vehicle(self, sysid: int):
        logging.info(f"Monitoring Vehicle {sysid}")
        intervention_sent = False
        intervention = self.interventions[sysid]
        trigger_seq = int(intervention.get("trigger_seq", 1)) if intervention else 0
        conn = self.conns[sysid]
        # This is a per-vehicle channel (self.conns[sysid] is its own UDP socket),
        # so every message read here belongs to this drone. Log them all to one
        # JSONL file per sysid — the GCS's ground-side view of the vehicle.
        telem_logger = DataLogger(path=DATA_PATH / "gcs_msgs", sysid=sysid)
        try:
            while True:
                msg = conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                msg_type = msg.get_type()

                telem_logger.write(
                    {
                        "type": "mavlink_in",
                        "msg_type": msg_type,
                        "data": make_json_safe(msg.to_dict()),
                        "time_received": time.time(),
                    }
                )

                # lat/lon 0,0 means the EKF has not converged yet — a
                # "no fix" marker rather than a position, and one that plots
                # ~3000 km from the origin if kept.
                if (
                    self.record_positions
                    and msg_type == "GLOBAL_POSITION_INT"
                    and not (msg.lat == 0 and msg.lon == 0)
                ):
                    self.paths[sysid].append(
                        GRA.from_global_int(msg.lat, msg.lon, msg.relative_alt)
                    )

                if msg_type == "STATUSTEXT" and msg.text == "LOGIC_DONE":
                    conn.mav.command_ack_send(
                        command=CustomCmd.LOGIC_DONE,
                        result=mavlink.MAV_RESULT_ACCEPTED,
                    )
                    logging.info(f"✅ Vehicle {sysid} completed its mission")
                    break

                if (
                    msg_type == "MISSION_CURRENT"
                    and intervention
                    and not intervention_sent
                    and msg.seq >= trigger_seq
                ):
                    self._send_intervention(sysid)
                    intervention_sent = True
        finally:
            self._remove_vehicle(sysid)
            logging.debug(f"Monitor thread finished for Vehicle {sysid}")

    def _send_intervention(self, sysid: int) -> None:
        iv = self.interventions[sysid]
        if iv is None:
            return
        cmd_conn = self.vehruntimes[sysid].cmd_conn
        logging.info(
            f"GCS intervention: switching vehicle {sysid} to GUIDED and repositioning"
        )
        cmd_conn.mav.set_mode_send(
            target_system=sysid,
            base_mode=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=4,  # GUIDED for ArduCopter
        )
        cmd_conn.mav.command_int_send(
            target_system=sysid,
            target_component=1,
            frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=192,  # MAV_CMD_DO_REPOSITION
            current=0,
            autocontinue=0,
            param1=-1.0,  # speed: no change
            param2=1.0,  # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
            param3=0.0,
            param4=float("nan"),
            x=int(iv["target_lat"] * 1e7),
            y=int(iv["target_lon"] * 1e7),
            z=float(iv["target_alt"]),
        )

    def _remove_vehicle(self, sysid: int):
        """Remove vehicles from the environment."""
        self.conns[sysid].close()
        del self.conns[sysid]
        del self.vehruntimes[sysid]
        del self.sysids[self.sysids.index(sysid)]
        self._terminate_veh_processes(sysid)
        self.n_vehicles -= 1
        logging.info(f"Vehicle {sysid} removed from GCS {self.name}")

    def _terminate_veh_processes(self, sysid: int) -> None:
        runtime = self.vehruntimes.get(sysid)
        if runtime is None:
            logging.debug(f"No runtime found for Vehicle {sysid}")
            return

        for name, proc in runtime.processes.items():
            terminate_process_group(proc, f"{name} for Vehicle {sysid}")

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
