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
from typing import Literal

import matplotlib.pyplot as plt
import zmq

from config import DATA_PATH, BasePort, Color
from helpers.connections.zeromq import create_zmq_sockets
from helpers.coordinates import GRA, GRAPose
from helpers.rid import RIDData


class RIDStore:
    """Thread-safe container for the latest RIDData and GRA position."""

    def __init__(self):
        self._rid: RIDData | None = None
        self._pos: GRA | None = None
        self._ready = threading.Event()
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def put(self, rid: RIDData):
        """
        Store the latest RIDData and GRA position,
        and notify any waiting threads.
        """
        if (
            self._stop.is_set()
            or (rid.lat is None)
            or (rid.lon is None)
            or (rid.alt is None)
        ):
            return
        with self._lock:
            self._pos = GRA(lat=rid.lat, lon=rid.lon, alt=rid.alt)
            self._rid = rid
        self._ready.set()

    def get(self) -> RIDData | None:
        """
        Wait for new data and retrieve the latest RIDData and GRA
        position.
        """
        if self._stop.is_set() or not self._ready.is_set():
            return None
        with self._lock:
            rid = self._rid
        return rid

    def pos(self) -> GRA | None:
        """Get a snapshot of the current RIDData and GRA position."""
        with self._lock:
            return self._pos

    def stop(self) -> None:
        """Mark store as stopped (e.g. UAV landed)."""
        self._stop.set()
        self._ready.set()  # wake any waiters so they don't block

    def is_stopped(self) -> bool:
        """Check if the store is marked as stopped."""
        return self._stop.is_set()


class GCSRIDStore:
    """Thread-safe container for the latest RIDData and GRA position."""

    def __init__(self, sysids: list[int]):
        self.rids = {sysid: RIDStore() for sysid in sysids}
        self._stop = threading.Event()

    def stop(self) -> None:
        """Mark store as stopped (e.g. UAV landed)."""
        self._stop.set()
        for store in self.rids.values():
            store.stop()

    def is_stopped(self) -> bool:
        """Check if the store is marked as stopped."""
        return self._stop.is_set()


class Oracle:  # UAVMonitor
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self,
        gra_origin: GRAPose,
        uav_port_offsets: dict[int, int],
        gcs_port_offsets: dict[str, int],
        gcs_sysids: dict[str, list[int]],
        transmission_range: float = 40.0,
    ) -> None:
        self.gra_origin = gra_origin
        self.gcs_sysids = gcs_sysids
        self.gcs_stores = {
            gcs: GCSRIDStore(sysids) for gcs, sysids in gcs_sysids.items()
        }
        self.rid_stores = {
            sysid: rid_store
            for gcs_store in self.gcs_stores.values()
            for sysid, rid_store in gcs_store.rids.items()
        }
        self.sysids = list(self.rid_stores.keys())
        self.range = transmission_range
        self.stop = threading.Event()

        zmq_ctx = zmq.Context()
        self.rid_in_socks = create_zmq_sockets(
            zmq_ctx, BasePort.RID_UP, zmq.SUB, uav_port_offsets
        )
        self.rid_out_socks = create_zmq_sockets(
            zmq_ctx, BasePort.RID_DOWN, zmq.PUB, uav_port_offsets
        )
        self.gcs_socks = create_zmq_sockets(
            zmq_ctx, BasePort.GCS_ZMQ, zmq.SUB, gcs_port_offsets
        )
        self.rid_in_threads = {
            sysid: threading.Thread(target=self.update_rid, args=(sysid,))
            for sysid in self.sysids
        }
        self.rid_out_threads = {
            sysid: threading.Thread(target=self.retransmit_rid, args=(sysid,))
            for sysid in self.sysids
        }

        self.gcs_threads = {
            name: threading.Thread(target=self.wait_gcs_done, args=(name,))
            for name in gcs_sysids.keys()
        }
        # Small delay to ensure ZMQ connections are established
        # time.sleep(0.2)

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
        for thread in self.gcs_threads.values():
            thread.start()
        logging.debug("Entering main monitoring loop...")

        while any(thread.is_alive() for thread in self.gcs_threads.values()):
            time.sleep(0.1)

        logging.info("✅ Main monitoring loop completed - all connections closed")

        logging.info("🎉 Oracle shutdown complete!")

    def wait_gcs_done(self, gcs_name: str):
        """Wait for a DONE message from one GCS, then stop its stores and threads."""
        while True:
            try:
                msg = self.gcs_socks[gcs_name].recv_string(flags=zmq.NOBLOCK)  # type: ignore
                if msg == "DONE":
                    self.gcs_stores[gcs_name].stop()
                    logging.info(f"Received DONE from GCS {gcs_name}")
                    for sysid in self.gcs_sysids[gcs_name]:
                        self.rid_in_threads[sysid].join()
                        self.rid_out_threads[sysid].join()
                    break
            except zmq.Again:
                # No message available, this is expected
                continue
            except Exception as e:
                logging.error(f"Error receiving from GCS {gcs_name}: {e}")
                continue
            time.sleep(0.01)

    def update_rid(self, sysid: int):
        """Receive Remote ID messages from one UAV and update the store."""
        store = self.rid_stores[sysid]
        sock = self.rid_in_socks[sysid]
        while not self.stop.is_set() and not store.is_stopped():
            try:
                msg: RIDData | Literal["DONE"] = self.rid_in_socks[sysid].recv_pyobj()  # type: ignore
                if msg == "DONE":
                    store.stop()
                    logging.info(f"Received DONE from UAV {sysid}")
                    break
                rid: RIDData = sock.recv_pyobj()  # type: ignore
                store.put(rid)
            except zmq.Again:
                continue
            except Exception as e:
                logging.error(f"RID error {sysid}: {e}")
            time.sleep(0.1)

    def retransmit_rid(self, sysid: int):
        """Retransmit Remote IDs to other UAVs."""
        store = self.rid_stores[sysid]
        while not self.stop.is_set() and not store.is_stopped():
            rid = store.get()
            if rid is None:
                continue
            pos = store.pos()
            if pos is None:
                continue
            # chechk if list can be taken out
            for other_sysid, other_rid_store in self.rid_stores.items():
                if other_sysid == sysid or other_rid_store.is_stopped():
                    continue
                other_pos = other_rid_store.pos()
                if other_pos is None:
                    continue
                dist = GRA.distance(pos, other_pos)
                logging.debug(
                    f"Distance between {sysid} and {other_sysid}: {dist:.1f} m"
                )
                if dist > self.range:
                    continue
                try:
                    self.rid_out_socks[other_sysid].send_pyobj(rid)  # type: ignore
                    logging.debug(
                        f"🔁 Retransmitted Remote ID {self.rid_stores[other_sysid]} "
                        f"from {sysid} to {other_sysid}"
                    )
                except Exception as e:
                    logging.error(
                        (
                            f"Error retransmitting Remote ID from {sysid} "
                            f"to {other_sysid}: {e}"
                        )
                    )

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
                enu = gra_origin.unpose().to_rel_all(gra_valid)
                ax.scatter(  # type: ignore
                    [p.x for p in enu],
                    [p.y for p in enu],
                    [p.z for p in enu],
                    c=[gcs_color.value],
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
