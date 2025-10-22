"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import logging
import pickle
import time
from concurrent import futures
from typing import TypedDict

import zmq
from pymavlink import mavutil

from config import DATA_PATH, ENV_CMD_ARP, ENV_CMD_PYT, BasePort
from helpers.connections.mavlink.conn import create_udp_conn
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.zeromq import create_zmq_socket
from helpers.coordinates import GRAs
from helpers.processes import create_process
from helpers.setup_log import setup_logging
from monitor import UAVMonitor
from params.simulation import HEARTBEAT_FREQUENCY

heartbeat_event = mavutil.periodic_event(HEARTBEAT_FREQUENCY)


def main():
    """Run a GCS instance to monitor UAVs."""
    config_path, verbose = parse_arguments()
    with open(config_path) as f:
        config = json.load(f)
    setup_logging(f"GCS_{config['name']}", verbose=verbose, console_output=True)
    gcs = GCS(**config)
    gcs.run()


class UAVGCSConfig(TypedDict):
    """TypedDict for UAV configuration in the GCS."""

    sysid: int
    port_offset: int
    ardupilot_cmd: str
    logic_cmd: str
    proxy_cmd: str


class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    port_offset: int
    uavs: list[UAVGCSConfig]
    terminals: list[str]
    suppress: list[str]


class GCS(UAVMonitor):
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(
        self,
        uavs: list[UAVGCSConfig],
        name: str,
        port_offset: int,
        terminals: list[str],
        suppress: list[str],
    ) -> None:
        # Configure logging for this GCS process
        self.name = name
        self.uavs = uavs
        self.sysids = [uavconfig["sysid"] for uavconfig in uavs]
        self.n_uavs = len(self.sysids)
        self.terminals = set(terminals)
        self.suppress = set(suppress)
        self.conns = self._launch_vehicles()  # NOTE: add self.conn =

        self.zmq_ctx = zmq.Context()
        self.orc_sock = create_zmq_socket(
            self.zmq_ctx, zmq.PUB, BasePort.GCS_ZMQ, port_offset
        )

        super().__init__(dict(zip(self.sysids, self.conns)))
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.sysids}

        logging.info(f" GCS {self.name} started with {self.n_uavs} UAVs")

    ###
    def run(self):
        """Run the GCS monitoring loop until all UAVs complete their missions."""
        with futures.ThreadPoolExecutor() as executor:
            executor.map(self._monitor_uav, self.sysids)

        logging.info("All UAVs assigned have completed their missions")
        self.orc_sock.send_string("DONE")  # type: ignore
        logging.info("DONE message sent to Oracle")
        self.orc_sock.close(linger=0)
        self.zmq_ctx.term()

        trajectory_file = DATA_PATH / f"trajectories_{self.name}.pkl"
        with open(trajectory_file, "wb") as file:
            pickle.dump(self.paths, file)
        logging.info(f"Trajectories saved to '{trajectory_file}'")

    def save_pos(self):
        """Save the current global position of each UAV to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)

    def _monitor_uav(self, sysid: int):
        logging.info(f"Monitoring UAV {sysid}")
        while not self.is_plan_done(sysid):
            self.get_global_pos(sysid)
            self.save_pos()
            # if heartbeat_period.trigger():
            #     send_heartbeat(self.conns[sysid])
            time.sleep(0.1)
        self.remove_uav(sysid)
        logging.info(f"UAV {sysid} mission completed")

    def _launch_vehicles(self) -> list[MAVConnection]:
        """Launch ArduPilot and logic processes for each UAV."""
        with futures.ThreadPoolExecutor() as executor:
            conns = list(executor.map(self._launch_uav, range(self.n_uavs)))
        return conns

    def _launch_uav(self, i: int):
        uav_config = self.uavs[i]
        sysid = uav_config["sysid"]

        p = create_process(
            uav_config["logic_cmd"],
            after="exec bash",
            visible="logic" in self.terminals,
            suppress_output="logic" in self.suppress,
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        logging.debug(f"UAV logic for vehicle {sysid} launched (PID {p.pid})")

        p = create_process(
            uav_config["proxy_cmd"],
            after="exec bash",
            visible="proxy" in self.terminals,
            suppress_output="proxy" in self.suppress,
            title=f"Proxy: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        logging.debug(f"Proxy for vehicle {sysid} launched (PID {p.pid})")

        p = create_process(
            uav_config["ardupilot_cmd"],
            after="exec bash",
            visible="launcher" in self.terminals,
            suppress_output="launcher" in self.suppress,
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        logging.debug(f"ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        conn = create_udp_conn(
            base_port=BasePort.GCS,
            offset=uav_config["port_offset"],
            mode="receiver",
        )
        logging.info(f"UAV {sysid} connected")
        return conn

    @staticmethod
    def load_config(config_path: str) -> GCSConfig:
        """Load GCS configuration from a JSON file via command line argument."""
        with open(config_path) as f:
            gcs_config: GCSConfig = json.load(f)
        return gcs_config


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
