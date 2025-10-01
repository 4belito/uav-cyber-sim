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
from mavlink.customtypes.location import GRAPose
from monitor import UAVMonitor


class Oracle(UAVMonitor):  # UAVMonitor
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
    ) -> None:
        self.pos: dict[int, GRA] = {}
        self.gcs_sysids = gcs_sysids
        self.sysids = list(uav_port_offsets)
        self.rid_queue = Queue[tuple[int, dict[str, float]]]()
        self.zmq_ctx = zmq.Context()

        self.rid_in_socks = dict[int, zmq.Socket[bytes]]()
        self.rid_out_socks = dict[int, zmq.Socket[bytes]]()
        for sysid, offset in uav_port_offsets.items():
            self.rid_in_socks[sysid] = self.zmq_ctx.socket(zmq.SUB)
            self.rid_in_socks[sysid].connect(
                f"tcp://127.0.0.1:{BasePort.RID_UP + offset}"
            )
            self.rid_in_socks[sysid].setsockopt_string(zmq.SUBSCRIBE, "")
            self.rid_in_socks[sysid].setsockopt(zmq.RCVTIMEO, 100)

            self.rid_out_socks[sysid] = self.zmq_ctx.socket(zmq.PUB)
            self.rid_out_socks[sysid].bind(
                f"tcp://127.0.0.1:{BasePort.RID_DOWN + offset}"
            )
            self.rid_out_socks[sysid].setsockopt(zmq.SNDTIMEO, 100)

        self.gcs_socks = dict[str, zmq.Socket[bytes]]()
        for gcs_name, offset in gcs_port_offsets.items():
            self.gcs_socks[gcs_name] = self.zmq_ctx.socket(zmq.SUB)
            self.gcs_socks[gcs_name].connect(
                f"tcp://127.0.0.1:{BasePort.GCS_ZMQ + offset}"
            )
            self.gcs_socks[gcs_name].setsockopt_string(zmq.SUBSCRIBE, "")
            self.gcs_socks[gcs_name].setsockopt(zmq.RCVTIMEO, 100)

        # Small delay to ensure ZMQ connections are established
        time.sleep(0.2)

    def run(self):
        """Run the Oracle to manage UAV connections and communication."""
        rid_in_threads = [
            threading.Thread(target=self.enqueue_remote_ids, args=(sysid,))
            for sysid in self.sysids
        ]
        rid_out_thread = threading.Thread(target=self.retransmit_remote_ids)

        logging.info(
            f"🏁 Starting Oracle with {len(self.sysids)} vehicles and "
            f"{len(self.gcs_socks)} GCSs"
        )

        for thread in rid_in_threads:
            thread.start()
        rid_out_thread.start()
        logging.debug("Entering main monitoring loop...")

        while self.sysids:
            for gcs_name, sock in list(self.gcs_socks.items()):
                try:
                    msg = sock.recv_string(flags=zmq.NOBLOCK)
                    logging.info(f"Received message '{msg}' from GCS {gcs_name}")
                    if msg == "DONE":
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

        for thread in rid_in_threads:
            thread.join()
        rid_out_thread.join()

        logging.info("🎉 Oracle shutdown complete!")

    def enqueue_remote_ids(self, sysid: int):
        """Receive Remote ID messages from one UAV and add them to the queue."""
        while sysid in self.sysids:
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
            self.rid_queue.put((sysid, rid))

    def retransmit_remote_ids(self):
        """Retransmit Remote IDs to other UAVs."""
        while self.sysids:
            try:
                sysid, rid = self.rid_queue.get(timeout=0.1)
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
                if dist > 100:
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
