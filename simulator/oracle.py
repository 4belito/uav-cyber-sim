"""
Define the Oracle class to simulate Vehicle-to-Vehicle communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate Vehicle-to-Vehicle communication.
Currently provides basic global position tracking and mission completion detection.
"""

from __future__ import annotations

import json
import logging
import pickle
import subprocess
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import zmq

from simulator.config import DATA_PATH, BasePort, Color
from simulator.entities import SimGCS, SimVehicle
from simulator.entities.riddata import RIDData
from simulator.helpers.connections import create_zmq_socket, create_zmq_sockets
from simulator.helpers.coordinates import GRAPose
from simulator.params.simulation import USE_NETWORK_SIM
from simulator.runtime.grid import Grid

TX_LOOP_SLEEP = 0.01
RX_LOOP_SLEEP = 0.10


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to Vehicle logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self,
        gra_origin: GRAPose,
        vehs: dict[int, SimVehicle],
        gcss: dict[str, SimGCS],
        port_offset: int,
        transmission_range: float = 40.0,
    ) -> None:
        # Narrow types for the type checker now that we've asserted no None values
        self.gra_origin = gra_origin.unpose()

        self.gcss = gcss
        self.sysids = list(vehs.keys())
        self.grid = Grid(cell_size=transmission_range * 1.01)
        self._seen_in_grid: set[int] = set()
        veh_port_offsets = {
            sysid: veh.port_offset_required for sysid, veh in vehs.items()
        }
        self.n_entities = len(self.sysids) + len(self.gcss)

        # Sockets
        zmq_ctx = zmq.Context()
        self.rid_in_socks = create_zmq_sockets(
            zmq_ctx, BasePort.RID_UP, zmq.SUB, veh_port_offsets
        )
        self.rid_out_socks = create_zmq_sockets(
            zmq_ctx, BasePort.RID_DOWN, zmq.PUB, veh_port_offsets
        )
        self.done_sock = create_zmq_socket(
            zmq_ctx, zmq.ROUTER, BasePort.ORC_DONE, offset=port_offset
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
        self.done_thread = threading.Thread(target=self.wait_done)

        # Events and locks for thread coordination
        self.stop_sys = {sysid: threading.Event() for sysid in self.sysids}
        self.stop_gcs = {gcs_name: threading.Event() for gcs_name in self.gcss.keys()}
        self.rid_locks = {sysid: threading.Lock() for sysid in self.sysids}

    def wait_done(self):
        """Wait for DONE messages from all Vehicles, then ACK and exit."""
        seen_done: set[str] = set()

        while len(seen_done) < self.n_entities:
            try:
                frames = self.done_sock.recv_multipart()
                identity = frames[0]
                msg = frames[-1]

                if msg == b"DONE":
                    sender = identity.decode()
                    if sender in seen_done:
                        self.done_sock.send_multipart([identity, b"ACK"])  # type: ignore
                        continue
                    seen_done.add(sender)
                    self.done_sock.send_multipart([identity, b"ACK"])  # type: ignore
                    sender_id = sender.split("-")
                    if sender_id[0] == "log":
                        sysid = int(sender_id[1])
                        self.stop_sys[sysid].set()
                        if sysid in self._seen_in_grid:
                            self.grid.remove_sysid(sysid)
                            logging.info(
                                f"Vehicle {sysid} completed mission and exited"
                            )
                    if sender_id[0] == "gcs":
                        name = sender_id[1]
                        self.stop_gcs[name].set()
                        logging.info(f"GCS {name} completed")
            except zmq.Again:
                continue
            except Exception as e:
                logging.error(f"Error receiving DONE: {e}")

    def run(self):
        """Run the Oracle to manage Vehicle connections and communication."""
        logging.info(
            f"🏁 Starting Oracle with {len(self.sysids)} vehicles and "
            f"{len(self.gcss)} GCSs"
        )

        for thread in self.rid_in_threads.values():
            thread.start()
        for thread in self.rid_out_threads.values():
            thread.start()
        self.done_thread.start()

        while any(not event.is_set() for event in self.stop_sys.values()):
            time.sleep(0.1)
        logging.info("✅ All Vehicle threads completed")
        while any(not event.is_set() for event in self.stop_gcs.values()):
            time.sleep(0.1)
        logging.info("✅ All GCS threads completed")

        logging.info("🎉 Oracle shutdown complete!")

    def update_rid(self, sysid: int):
        """Receive Remote ID messages from one Vehicle and update the store."""
        while not self.stop_sys[sysid].is_set():
            try:
                rid: RIDData = self.rid_in_socks[sysid].recv_pyobj()
                if sysid in self._seen_in_grid:
                    self.grid.update(sysid, rid)
                else:
                    self.grid.add_rid(sysid, rid)
                    self._seen_in_grid.add(sysid)
            except zmq.Again:
                pass
            except Exception as e:
                logging.error(f"RID error {sysid}: {e}")
            time.sleep(RX_LOOP_SLEEP)

    def retransmit_rid(self, sysid: int):
        """Retransmit Remote IDs to neighbor Vehicles (one-shot per update)."""
        while not self.stop_sys[sysid].is_set():
            if sysid not in self._seen_in_grid:
                time.sleep(TX_LOOP_SLEEP)
                continue
            try:
                rid = self.grid.pop_rid(sysid)
                if rid is None:
                    time.sleep(TX_LOOP_SLEEP)
                    continue
                # get position and velocity parameters for each drone
                if USE_NETWORK_SIM:
                    pos = rid.enu_pos
                    spd = rid.speed
                    cog = rid.cog
                    ele = rid.ele
                    operands = [
                        (
                            f"{sysid},{round(pos.x, 3)},{round(pos.y, 3)},"
                            f"{round(pos.z, 3)},{round(spd, 3)},"
                            f"{round(cog, 3)},{round(ele, 3)}"
                        )
                    ]
                    o_sysids: list[int] = []
                    for o_sysid in self.grid.iter_neighbors_within(
                        sysid, rid.enu_pos, radius=None
                    ):
                        o_rid = self.grid.rid(o_sysid)
                        logging.debug(
                            f"{sysid}: {rid.enu_pos} -> {o_sysid}: {o_rid.enu_pos}"
                        )
                        o_sysids.append(o_sysid)
                        o_pos = o_rid.enu_pos
                        o_spd = o_rid.speed
                        o_cog = o_rid.cog
                        o_ele = o_rid.ele
                        operands.append(
                            (
                                f"{o_sysid},{round(o_pos.x, 3)},{round(o_pos.y, 3)},"
                                f"{round(o_pos.z, 3)},{round(o_spd, 3)},"
                                f"{round(o_cog, 3)},{round(o_ele, 3)}"
                            )
                        )

                    # continue if there not at least two drones to simulate
                    if len(operands) <= 1:
                        continue

                    # invoke a one-off uli-net-sim Remote ID broadcast simulation
                    result = subprocess.run(
                        [
                            "./rid-one-off.sh",
                            "-n",
                            f"{sysid}",
                            # TODO: fill in these RID fields later if needed
                            "-t",
                            "0",
                            "-x",
                            "0",
                            "-y",
                            "0",
                            "-z",
                            "0",
                            "-v",
                            "0",
                            "-g",
                            "0",
                            "-h",
                            "0",
                            "-q",
                            "--",
                            *operands,
                        ],
                        cwd="/usr/uli-net-sim",
                        capture_output=True,
                        text=True,
                    )
                    logging.debug(
                        f"rid-one-off:\noperands:\n{operands}\n\nstdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
                    )
                    res = {}
                    if result.stdout != "":
                        res = json.loads(result.stdout)
                    for o_sysid in o_sysids:
                        if "Serial Number" in res:
                            if str(o_sysid) in res["Serial Number"]:
                                if (
                                    str(sysid)
                                    in res["Serial Number"][str(o_sysid)]["values"]
                                ):
                                    with self.rid_locks[o_sysid]:
                                        self.rid_out_socks[o_sysid].send_pyobj(rid)  # type: ignore
                else:
                    for o_sysid in self.grid.iter_neighbors_within(
                        sysid, rid.enu_pos, radius=None
                    ):
                        with self.rid_locks[o_sysid]:
                            self.rid_out_socks[o_sysid].send_pyobj(rid)  # type: ignore
            except Exception as e:
                logging.error(f"Retransmit error for {sysid} of type {type(e)}: {e}")
            time.sleep(TX_LOOP_SLEEP)

    @staticmethod
    def plot_trajectories(gra_origin: GRAPose):
        """Plot trajectories of Vehicles for each GCS color."""
        traj_files = list(Path(DATA_PATH).glob("trajectories_*.pkl"))
        for file in traj_files:
            with open(file, "rb") as f:
                trajs = pickle.load(f)
            # Extract color name from filename: trajectories_COLOR_EMOJI.pkl
            stem_parts = file.stem.split("_")
            color_name = stem_parts[1]
            if color_name.lower() not in Color.__members__:
                gcs_color = Color.BLACK  # Default to gray if color not recognized
            else:
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
                    label=f"Vehicle {sysid}",
                    depthshade=True,
                )
            ax.set_aspect(aspect="equalxy")  # type: ignore
            plt.tight_layout()
        plt.show(block=True)  # type: ignore

    def wait_for_trajectory_files(self, poll_interval: float = 0.1):
        """Wait until n_expected trajectory files exist in DATA_PATH, or timeout."""
        n_expected = len(self.gcss)
        while True:
            traj_files = list(Path(DATA_PATH).glob("trajectories_*.pkl"))
            if len(traj_files) == n_expected:
                logging.info(f"Found {len(traj_files)} trajectory files")
                return True
            time.sleep(poll_interval)
