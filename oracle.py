"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

import logging
import pickle
import threading
import time
from pathlib import Path
from queue import Queue

import matplotlib.pyplot as plt
import zmq

from config import DATA_PATH, BasePort, Color
from helpers.change_coordinates import GRA, GLOBAL_INT_to_GRA, GRAs_to_ENUs
from helpers.connections.mavlink.customtypes.location import GRAPose
from helpers.connections.zeromq import create_zmq_sockets


class Oracle:  # UAVMonitor
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self,
        uav_port_offsets: dict[int, int],
        gcs_port_offsets: dict[str, int],
        gcs_sysids: dict[str, list[int]],
        transmission_range: float = 100.0,
    ) -> None:
        self.gcs_sysids = gcs_sysids
        self.sysids = list(uav_port_offsets)
        self.pos: dict[int, GRA | None] = {sysid: None for sysid in self.sysids}
        self.sysids_lock = threading.Lock()
        self.rid_queues = {sysid: Queue[dict[str, float]]() for sysid in self.sysids}
        self.zmq_ctx = zmq.Context()
        self.range = transmission_range

        self.rid_in_socks = create_zmq_sockets(
            self.zmq_ctx, BasePort.RID_UP, zmq.SUB, uav_port_offsets
        )
        self.rid_out_socks = create_zmq_sockets(
            self.zmq_ctx, BasePort.RID_DOWN, zmq.PUB, uav_port_offsets
        )
        self.gcs_socks = create_zmq_sockets(
            self.zmq_ctx, BasePort.GCS_ZMQ, zmq.SUB, gcs_port_offsets
        )
        self.rid_in_threads = {
            sysid: threading.Thread(target=self.enqueue_remote_ids, args=(sysid,))
            for sysid in self.sysids
        }
        self.rid_out_threads = {
            sysid: threading.Thread(target=self.retransmit_remote_ids, args=(sysid,))
            for sysid in self.sysids
        }
        # Small delay to ensure ZMQ connections are established
        time.sleep(0.2)

    def run(self):
        """Run the Oracle to manage UAV connections and communication."""
        logging.info(
            f"🏁 Starting Oracle with {len(self.sysids)} vehicles and "
            f"{len(self.gcs_socks)} GCSs"
        )

        for thread in self.rid_in_threads.values():
            thread.start()
        for thread in self.rid_out_threads.values():
            thread.start()
        logging.debug("Entering main monitoring loop...")

        while self.sysids:
            for gcs_name, sock in list(self.gcs_socks.items()):
                try:
                    msg = sock.recv_string(flags=zmq.NOBLOCK)
                    logging.info(f"Received message '{msg}' from GCS {gcs_name}")
                    if msg == "DONE":
                        with self.sysids_lock:
                            self.remove_gcs(gcs_name)
                    else:
                        logging.debug(f"Ignoring non-DONE message: {msg}")
                except zmq.Again:
                    # No message available, this is expected
                    continue
                except Exception as e:
                    logging.error(f"Error receiving from GCS {gcs_name}: {e}")
                    continue

        logging.info("✅ Main monitoring loop completed - all connections closed")

        logging.info("🎉 Oracle shutdown complete!")

    def enqueue_remote_ids(self, sysid: int):
        """Receive Remote ID messages from one UAV and add them to the queue."""
        while sysid not in self.sysids:
            try:
                rid: dict[str, float] = self.rid_in_socks[sysid].recv_json()  # type: ignore
                raw_lat = rid.get("lat")
                raw_lon = rid.get("lon")
                raw_alt = rid.get("alt")
                if raw_lat and raw_lon and raw_alt:
                    self.pos[sysid] = GLOBAL_INT_to_GRA(raw_lat, raw_lon, raw_alt)
                logging.debug(f"Remote ID from {sysid} and msg {rid}")
            except zmq.Again:
                continue
            except Exception:
                logging.error(f"Error receiving Remote ID from {sysid}")
                continue
            self.rid_queues[sysid].put(rid)

    def retransmit_remote_ids(self, sysid: int):
        """Retransmit Remote IDs to other UAVs."""
        with self.sysids_lock:
            while sysid not in self.sysids:
                try:
                    rid = self.rid_queues[sysid].get(timeout=0.1)
                except Exception:
                    continue
                logging.debug(f"🔁 Received Remote ID from {sysid}")
                pos = self.pos.get(sysid, None)
                if pos is None:
                    continue
                for other_sysid, other_sock in list(self.rid_out_socks.items()):
                    if other_sysid == sysid:
                        continue
                    other_pos = self.pos.get(other_sysid, None)
                    if other_pos is None:
                        continue
                    dist = GRA.distance(pos, other_pos)
                    if dist > self.range:
                        continue
                    try:
                        other_sock.send_json(rid)  # type: ignore
                        logging.debug(
                            f"🔁 Retransmitted Remote ID from {sysid} to {other_sysid}"
                        )
                    except Exception as e:
                        logging.error(
                            (
                                f"Error retransmitting Remote ID from {sysid} "
                                f"to {other_sysid}: {e}"
                            )
                        )

    def remove_gcs(self, gcs_name: str):
        """Remove a GCS connection and its associated sockets."""
        logging.debug(
            f"Removing GCS {gcs_name} and its UAVs: {self.gcs_sysids[gcs_name]}"
        )
        del self.gcs_socks[gcs_name]
        for sysid in self.gcs_sysids[gcs_name]:
            logging.debug(f"Removing UAV {sysid} from tracking")
            self.remove_uav(sysid)
        logging.info(f"GCS {gcs_name} removED. Remaining GCS: {len(self.gcs_socks)}")

    def remove_uav(self, sysid: int):
        """Remove a UAV connection and its associated sockets."""
        # super().remove_uav(sysid)
        self.sysids.remove(sysid)
        self.rid_in_threads[sysid].join()
        self.rid_out_threads[sysid].join()
        self.rid_queues[sysid].queue.clear()
        del self.rid_queues[sysid]
        del self.rid_in_socks[sysid]
        del self.rid_out_socks[sysid]

    @staticmethod
    def plot_trajectories(gra_origin: GRAPose):
        """Plot trajectories of UAVs for each GCS color."""
        traj_files = list(Path(DATA_PATH).glob("trajectories_*.pkl"))
        for file in traj_files:
            with open(file, "rb") as f:
                trajs = pickle.load(f)
            # Extract color name from filename: trajectories_COLOR_EMOJI.pkl
            stem_parts = file.stem.split("_")
            color_name = stem_parts[1]
            gcs_color = Color(color_name.lower())
            fig = plt.figure(figsize=(8, 8))  # type: ignore
            ax = fig.add_subplot(projection="3d", proj_type="ortho")  # type: ignore
            ax.set_title(f"{gcs_color} ENU Trajectories")  # type: ignore
            ax.set_xlabel("East (m)")  # type: ignore
            ax.set_ylabel("North (m)")  # type: ignore
            ax.set_zlabel("Up (m)")  # type: ignore

            for sysid, gra_path in trajs.items():
                gra_valid = [p for p in gra_path if abs(p.alt) > 0.5]
                enu = GRAs_to_ENUs(GRA(*gra_origin[:3]), gra_valid)
                xs = [p.x for p in enu]
                ys = [p.y for p in enu]
                zs = [p.z for p in enu]
                ax.scatter(  # type: ignore
                    xs,
                    ys,
                    zs,
                    c=[gcs_color.value],  # Use the actual color value
                    s=12,  # type: ignore
                    alpha=0.8,
                    label=f"UAV {sysid}",
                    depthshade=True,
                )
            ax.set_aspect(aspect="equalxy")  # type: ignore
            plt.tight_layout()
        plt.show(block=True)  # type: ignore

    def wait_for_trajectory_files(self, poll_interval: float = 0.1):
        """Wait until n_expected trajectory files exist in DATA_PATH, or timeout."""
        n_expected = len(self.gcs_sysids)
        while True:
            traj_files = list(Path(DATA_PATH).glob("trajectories_*.pkl"))
            if len(traj_files) == n_expected:
                logging.info(f"Found {len(traj_files)} trajectory files")
                return True
            time.sleep(poll_interval)
