"""
Define the GCS class to monitor UAVs through MAVLink messages and run GCS
instances.
"""

import argparse
import ast
import pickle

from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import BasePort
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import GRAs
from oracle import Oracle


def main():
    """Run a GCS instance to monitor UAVs."""
    gcs_name, system_ids, port_offsets = parse_arguments()
    conns: dict[int, MAVConnection] = {}
    for sysid, port_offset in zip(system_ids, port_offsets):
        port = BasePort.GCS + port_offset
        conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
        conn.wait_heartbeat()
        print(f"🔗 UAV logic {sysid} is connected")
        conns[sysid] = conn
    gcs = GCS(conns, gcs_name)
    while len(gcs.conns):
        gcs.gather_broadcasts()
        gcs.save_pos()
        for sysid in list(gcs.conns.keys()):
            if gcs.is_plan_done(sysid):
                gcs.remove(sysid)
    print(f"✅ All UAVs assigned to GCS {gcs_name} have completed their mission.")
    trajectory_file = f"trajectories_{gcs_name}.pkl"
    with open(trajectory_file, "wb") as file:
        pickle.dump(gcs.paths, file)

    print(f"💾 Trajectories saved to '{trajectory_file}'.")


class GCS(Oracle):
    """Ground Control Station class extending Oracle with trajectory logging."""

    def __init__(self, conns: dict[int, MAVConnection], name: str = "blue 🟦"):
        self.name = name
        super().__init__(conns, name=f"GCS {name}")
        self.paths: dict[int, GRAs] = {sysid: [] for sysid in self.conns}

    def save_pos(self):
        """Save the current global position of each UAV to their trajectory path."""
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)


def parse_arguments() -> tuple[str, list[int], list[int]]:
    """Parse List of GCS system IDs and GCS name."""
    parser = argparse.ArgumentParser(description="Single GCS")
    parser.add_argument(
        "--sysids",
        type=ast.literal_eval,  # convert test to python (list)
        required=True,
        help='ystem ID Lsit of the UAVs belonging to the GCS (e.g. "[1,3,4]")',
    )
    parser.add_argument(
        "--port-offsets",
        type=ast.literal_eval,
        required=True,
        help='Port offset list for the UAVs belonging to the GCS (e.g. "[0,10,30]")',
    )
    parser.add_argument(
        "--name",
        type=str,
        required=True,
        help="System ID Lsit of the UAVs belonging to the GCS (e.g., [1, 3,4])",
    )
    args = parser.parse_args()
    return (
        args.name,
        args.sysids,
        args.port_offsets,
    )


if __name__ == "__main__":
    main()
