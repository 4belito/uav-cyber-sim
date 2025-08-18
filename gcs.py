"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import logging
import pickle
from concurrent import futures
from typing import TypedDict

import zmq

from config import DATA_PATH, ENV_CMD_ARP, ENV_CMD_PYT, BasePort
from helpers.processes import create_process
from helpers.setup_log import setup_logging
from mavlink.customtypes.location import GRAs
from monitor import UAVMonitor
from proxy import create_udp_conn


def main():
    """Run a GCS instance to monitor UAVs."""
    config_path, verbose = parse_arguments()
    gcs = GCS(config_path, verbose=verbose)
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

    def __init__(self, config_path: str, verbose: int = 1):
        # Configure logging for this GCS process

        with open(config_path) as f:
            self.config: GCSConfig = json.load(f)
        self.verbose = verbose
        self.n_uavs = len(self.config["uavs"])
        self.terminals = set(self.config["terminals"])
        self.suppress = set(self.config["suppress"])
        self._launch_vehicles()

        self.zmq_ctx = zmq.Context()
        self.orc_sock = self.zmq_ctx.socket(zmq.PUB)
        self.orc_sock.bind(
            f"tcp://127.0.0.1:{BasePort.GCS_ZMQ + self.config['port_offset']}"
        )
        self.orc_sock.setsockopt(zmq.SNDTIMEO, 100)

        super().__init__(self.conns, name=f"GCS {self.config['name']}", verbose=verbose)
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.conns}
        setup_logging(self.config["name"], verbose=verbose, console_output=True)
        logging.debug(f"{self.config['name']} GCS initialized with {self.n_uavs} UAVs")
        logging.info(f"GCS {self.config['name']} started with {self.n_uavs} UAVs")

    def run(self):
        """Run the GCS monitoring loop until all UAVs complete their missions."""
        while len(self.conns):
            self.gather_broadcasts()
            self.save_pos()
            for sysid in list(self.conns.keys()):
                if self.is_plan_done(sysid):
                    self.remove_uav(sysid)

        logging.info(
            f"All UAVs assigned to GCS {self.config['name']} have completed their mission"
        )
        logging.info(f"GCS {self.config['name']}: Sending DONE message to Oracle...")
        self.orc_sock.send_string("DONE")  # type: ignore
        logging.info(f"GCS {self.config['name']}: DONE message sent to Oracle")

        # Small delay to ensure the message is sent before process termination
        import time

        time.sleep(0.1)

        trajectory_file = DATA_PATH / f"trajectories_{self.config['name']}.pkl"
        with open(trajectory_file, "wb") as file:
            pickle.dump(self.paths, file)
        logging.info(f"Trajectories saved to '{trajectory_file}'")

    def save_pos(self):
        """Save the current global position of each UAV to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)

    def _launch_vehicles(self):
        """Launch ArduPilot and logic processes for each UAV."""
        with futures.ThreadPoolExecutor() as executor:
            self.conns = dict(
                zip(
                    range(self.n_uavs),
                    executor.map(self._launch_uav, range(self.n_uavs)),
                )
            )

    def _launch_uav(self, i: int):
        uav_config = self.config["uavs"][i]
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
        logging.info(f"UAV {sysid} connected to GCS {self.config['name']}")
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
