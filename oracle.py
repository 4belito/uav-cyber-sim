"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

from __future__ import annotations

import logging
import pickle
import threading
import time
from collections import defaultdict
from collections.abc import Iterable
from math import floor
from pathlib import Path
from typing import Dict, Literal

import matplotlib.pyplot as plt
import zmq

from config import DATA_PATH, BasePort, Color
from helpers.connections.zeromq import create_zmq_sockets
from helpers.coordinates import ENU, GRA, GRAPose
from helpers.rid import RIDData

# from params.simulation import REMOTE_ID_FREQUENCY

CellKey = tuple[int, int, int]


class RID:
    """Thread-safe container for the latest RIDData and GRA position."""

    def __init__(self, riddata: RIDData, gra_origin: GRA):
        self._pos: ENU = self.ridata2pos(riddata, gra_origin)
        self._data: RIDData | None = riddata
        # self._pending: RIDData | None = riddata
        self.lock = threading.Lock()
        # self._event = mavutil.periodic_event(REMOTE_ID_FREQUENCY)

    @staticmethod
    def ridata2pos(rid: RIDData, origin: GRA) -> ENU:
        """Convert RIDData to ENU position using the given GRA origin."""
        if rid.lat is None or rid.lon is None or rid.alt is None:
            raise ValueError("RIDData has None for lat, lon, or alt")
        return origin.to_rel(GRA(rid.lat, rid.lon, rid.alt))  # type: ignore

    def put(self, riddata: RIDData, gra_origin: GRA) -> None:
        """
        Store the latest RIDData and GRA position,
        and notify any waiting threads.
        """
        pos = self.ridata2pos(riddata, gra_origin)  # type: ignore
        with self.lock:
            self._pos = pos
            self._data = riddata

    # def data(self) -> RIDData:
    #     """
    #     Wait for new data and retrieve the latest RIDData and GRA
    #     position.
    #     """
    #     with self.lock:
    #         return self._data

    def pos(self) -> ENU:
        """Get a snapshot of the current RIDData and GRA position."""
        with self.lock:
            return self._pos

    def pop_data(self) -> RIDData | None:
        """Return the pending RIDData once and clear it so it won't be resent."""
        with self.lock:
            pkt = self._data
            self._data = None
            return pkt

    def pending(self) -> bool:
        """Check if there is pending RIDData to be sent."""
        with self.lock:
            return self._data is not None


Cell = list[int]


class Grid:
    """Lightweight 3D spatial index for neighbor queries."""

    def __init__(self, cell_size: float) -> None:
        assert cell_size > 0
        self.cell_size = cell_size
        self._cell: Dict[CellKey, Cell] = defaultdict(list)
        self._rid: Dict[int, RID] = {}
        # map sysid -> cellkey
        self._key: Dict[int, CellKey] = {}
        self._lock = threading.RLock()  # single structure lock

    # === Core methods ===
    def _idx(self, coor: float) -> int:
        return int(floor(coor / self.cell_size))

    def _pos2key(self, pos: ENU) -> CellKey:
        return (self._idx(pos.x), self._idx(pos.y), self._idx(pos.z))

    def get_rid(self, sysid: int) -> RID | None:
        """Get the RID object for a given sysid, or False if not found or not pending."""
        with self._lock:
            rid = self._rid.get(sysid)
            if rid is not None and rid.pending():
                return rid
            return None

    # === Sysid management ===
    def add_rid(self, sysid: int, rid: RID) -> None:
        """
        Insert a new UAV at the given position.
        It assumes sysid is not already present.
        """
        pos = rid.pos()
        k = self._pos2key(pos)
        with self._lock:
            if sysid in self._rid:
                raise ValueError(f"sysid {sysid} already exists in grid")
            self._cell[k].append(sysid)
            self._key[sysid] = k
            self._rid[sysid] = rid

    def remove_sysid(self, sysid: int) -> None:
        """Completely remove a UAV from the grid."""
        with self._lock:
            k = self._key.pop(sysid, None)
            self._rid.pop(sysid, None)
            if k is None:
                return
            cell = self._cell.get(k)
            if cell:
                try:
                    cell.remove(sysid)
                except ValueError:
                    pass
                if not cell:
                    self._cell.pop(k, None)

    def update(self, sysid: int, rid: RID) -> None:
        """Incrementally move sysid between cells if needed."""
        pos = rid.pos()
        k_new = self._pos2key(pos)
        with self._lock:
            k_old = self._key.get(sysid)
            if k_old != k_new:
                # Remove from old cell (if any), avoiding accidental creation
                if k_old is not None:
                    old_cell = self._cell.get(k_old)  # get avoids new cell
                    if old_cell:
                        try:
                            old_cell.remove(sysid)
                        except ValueError:
                            pass
                        if not old_cell:
                            self._cell.pop(k_old, None)
                # Add to new cell
                self._cell[k_new].append(sysid)
                self._key[sysid] = k_new
            self._rid[sysid] = rid

    # === Neighbor queries ===
    def _iter_neighbor_keys(self, pos: ENU) -> Iterable[CellKey]:
        cx, cy, cz = self._pos2key(pos)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (cx + dx, cy + dy, cz + dz)
                    if key in self._cell:
                        yield key

    def iter_neighbor_sysids_snapshot(self, pos: ENU) -> list[int]:
        """Return a snapshot of sysids in 3x3x3 neighborhood (safe, no locks held)."""
        with self._lock:
            cell: list[int] = []
            for key in self._iter_neighbor_keys(pos):
                cell.extend(self._cell[key])
            return cell

    def iter_neighbors_within(
        self, sysid: int, pos: ENU, radius: float | None = None
    ) -> Iterable[int]:
        """
        Yield neighbor sysids within optional Euclidean radius.
        Assumes radius = None or 0<radius<=cell_size for correctness.
        """
        r2 = None if radius is None else radius * radius
        neighbor_ids = self.iter_neighbor_sysids_snapshot(pos)
        for o_sysid in neighbor_ids:
            if o_sysid == sysid:
                continue
            with self._lock:
                o_rid = self._rid.get(o_sysid)
            if o_rid is None:
                continue
            if r2 is not None and ENU.distance_squared(pos, o_rid.pos()) > r2:
                continue
            yield o_sysid


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
        self.gra_origin = gra_origin.unpose()
        self.gcs_sysids = gcs_sysids
        self.sysids = list(uav_port_offsets.keys())
        self.grid = Grid(cell_size=transmission_range * 1.01)
        self._seen_in_grid: set[int] = set()

        # Sockers
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

        # Threads
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
        logging.info("✅ All background threads started")

        # while any(thread.is_alive() for thread in self.rid_in_threads.values()):
        #     time.sleep(0.1)
        # logging.info("✅ All RID input threads completed")
        # while any(thread.is_alive() for thread in self.rid_out_threads.values()):
        #     time.sleep(0.1)
        # logging.info("✅ All RID output threads completed")

        while any(thread.is_alive() for thread in self.gcs_threads.values()):
            time.sleep(0.1)
        logging.info("✅ All GCS threads completed")

        logging.info("🎉 Oracle shutdown complete!")

    def wait_gcs_done(self, gcs_name: str):
        """Wait for a DONE message from one GCS, then stop all UAV threads."""
        while True:
            try:
                msg = self.gcs_socks[gcs_name].recv_string(flags=zmq.NOBLOCK)  # type: ignore
                if msg == "DONE":
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
        while True:
            try:
                msg: RIDData | Literal["DONE"] = self.rid_in_socks[sysid].recv_pyobj()  # type: ignore
                if msg == "DONE":
                    if sysid in self._seen_in_grid:
                        self.grid.remove_sysid(sysid)
                    logging.info(f"Received DONE from UAV {sysid}")
                    break
                logging.debug(f"Received rid msg {msg}")
                rid = RID(msg, self.gra_origin)
                if sysid in self._seen_in_grid:
                    self.grid.update(sysid, rid)
                else:
                    self.grid.add_rid(sysid, rid)
                    self._seen_in_grid.add(sysid)
            except zmq.Again:
                continue
            except Exception as e:
                logging.error(f"RID error {sysid}: {e}")
            time.sleep(0.1)

    def retransmit_rid(self, sysid: int):
        """Retransmit Remote IDs to other UAVs."""
        while self.rid_in_threads[sysid].is_alive():
            if sysid not in self._seen_in_grid:
                time.sleep(0.01)
                continue
            try:
                rid = self.grid.get_rid(sysid)
                if rid is None:
                    time.sleep(0.01)
                    continue
                pkt = rid.pop_data()
                logging.debug(f"Retransmit check for {sysid}: {rid}")
                for o_sysid in self.grid.iter_neighbors_within(
                    sysid, rid.pos(), radius=None
                ):
                    self.rid_out_socks[o_sysid].send_pyobj(pkt)  # type: ignore
            except Exception as e:
                logging.error(f"Retransmit error for {sysid}: {e}")
            time.sleep(0.01)

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
