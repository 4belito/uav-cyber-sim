"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
import json
import pickle
from typing import TypedDict

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import DATA_PATH, BasePort
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import GRAs
from oracle import Oracle


def main():
    """Run a GCS instance to monitor UAVs."""
    config_path, verbose = parse_arguments()
    config = GCS.load_config(config_path)
    gcs_name = config["name"]
    gcs_uavs = config["uavs"]
    conns: dict[int, MAVConnection] = {}
    for uav in gcs_uavs:
        sysid = uav["sysid"]
        port_offset = uav["port_offset"]
        port = BasePort.GCS + port_offset
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        if verbose:
            print(f"🔗 UAV logic {sysid} is connected")
        conns[sysid] = conn
    gcs = GCS(conns, gcs_name, verbose)
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


class GCSConfig(TypedDict):
    """Ground Control Station (GCS) Configuration."""

    name: str
    uavs: list[UAVGCSConfig]


class GCS(Oracle):
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(
        self, conns: dict[int, MAVConnection], name: str = "blue 🟦", verbose: int = 1
    ):
        self.name = name
        super().__init__(conns, name=f"GCS {name}", verbose=verbose)
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.conns}

    def save_pos(self):
        """Save the current global position of each UAV to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)

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
