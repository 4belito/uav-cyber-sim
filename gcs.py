"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
from concurrent import futures
import json
import pickle
from typing import TypedDict

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import DATA_PATH, ENV_CMD_ARP, ENV_CMD_PYT, BasePort
from helpers.processes import create_process
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import GRAs
from monitor import UAVMonitor


def main():
    """Run a GCS instance to monitor UAVs."""
    config_path, verbose = parse_arguments()
    gcs = GCS(config_path, verbose=verbose)
    gcs_name = gcs.config["name"]
    while len(gcs.conns):
        gcs.gather_broadcasts()
        gcs.save_pos()
        for sysid in list(gcs.conns.keys()):
            if gcs.is_plan_done(sysid):
                gcs.remove(sysid)
    if verbose:
        print(f"✅ All UAVs assigned to GCS {gcs_name} have completed their mission.")
    trajectory_file = DATA_PATH / f"trajectories_{gcs_name}.pkl"
    with open(trajectory_file, "wb") as file:
        pickle.dump(gcs.paths, file)
    if verbose:
        print(f"💾 Trajectories saved to '{trajectory_file}'.")


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
    uavs: list[UAVGCSConfig]
    terminals: list[str]
    suppress: list[str]


class GCS(UAVMonitor):
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(self, config_path: str, verbose: int = 1):
        with open(config_path) as f:
            self.config: GCSConfig = json.load(f)
        self.verbose = verbose
        self.n_uavs = len(self.config["uavs"])
        self.terminals = set(self.config["terminals"])
        self.suppress = set(self.config["suppress"])
        self._launch_vehicles()
        super().__init__(self.conns, name=f"GCS {self.config['name']}", verbose=verbose)
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.conns}

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
            uav_config["ardupilot_cmd"],
            after="exec bash",
            visible="launcher" in self.terminals,
            suppress_output="launcher" in self.suppress,
            title=f"ArduPilot SITL Launcher: Vehicle {sysid}",
            env_cmd=ENV_CMD_ARP,
        )  # "exit"
        if self.verbose:
            print(f"🚀 ArduPilot SITL vehicle {sysid} launched (PID {p.pid})")

        p = create_process(
            uav_config["logic_cmd"],
            after="exec bash",
            visible="logic" in self.terminals,
            suppress_output="logic" in self.suppress,
            title=f"UAV logic: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        if self.verbose:
            print(f"🚀 UAV logic for vehicle {sysid} launched (PID {p.pid})")

        p = create_process(
            uav_config["proxy_cmd"],
            after="exec bash",
            visible="proxy" in self.terminals,
            suppress_output="proxy" in self.suppress,
            title=f"Proxy: Vehicle {sysid}",
            env_cmd=ENV_CMD_PYT,
        )  # "exit"
        if self.verbose:
            print(f"🚀 Proxy for vehicle {sysid} launched (PID {p.pid})")

        port = BasePort.GCS + uav_config["port_offset"]
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        if self.verbose:
            print(f"🔗 UAV {sysid} is connected to GCS {self.config['name']}")
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
        help="Path to the GCS configuration file (e.g. gcs_config_blue.json)",
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
