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
from collections.abc import Iterable
from pathlib import Path

import matplotlib.pyplot as plt
import zmq
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from simulator.config import DATA_PATH, Color, SimPort, VehPort
from simulator.configs.mitm import MITMConfig
from simulator.entities import SimGCS, SimVehicle
from simulator.entities.riddata import RIDData
from simulator.helpers.connections import create_zmq_socket, create_zmq_sockets
from simulator.helpers.coordinates import GRA, ENUs, GRAPose, GRAs
from simulator.params.simulation import USE_NETWORK_SIM
from simulator.runtime.grid import Grid

# Smallest ground span a plot is given, so a stationary or straight-line flight
# is not zoomed into centimetres of numerical noise.
MIN_PLOT_SPAN = 5.0  # metres
# The Up axis gets a much smaller floor: altitude detail is worth seeing, and a
# 5 m floor would bury a low hop or the centimetre-scale wobble of a landed
# vehicle. Only a genuinely flat track is padded to this.
MIN_UP_SPAN = 1.0  # metres

TX_LOOP_SLEEP = 0.01
RX_LOOP_SLEEP = 0.10

# Module-level registry so clean() can reach active Oracle instances
_active: set["Oracle"] = set()


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to Vehicle logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self, transmission_range: float = 100.0, record_positions: bool = True
    ) -> None:
        """
        Configure an Oracle. It is not usable until `bind` supplies the
        launch-time wiring, which `Simulator.launch()` does for you.

        `transmission_range` is the inter-vehicle Remote ID range, in metres.
        `record_positions` keeps each vehicle's Remote ID track for
        `plot_trajectories`; it covers every vehicle, GCS-monitored or not.
        """
        self.transmission_range = transmission_range
        self.record_positions = record_positions
        # sysid -> ENU track, filled from Remote ID as the run proceeds.
        self.paths: dict[int, ENUs] = {}
        self.grid = Grid(cell_size=transmission_range * 1.01)
        self._seen_in_grid: set[int] = set()
        self._bound = False

        # --- the scenario: what is being simulated -------------------------
        # Owned here rather than by the Simulator, which only decides how the
        # scenario is executed (ports, processes, visualizer).
        self.vehicles: dict[int, SimVehicle] = {}
        self.gcss: dict[str, SimGCS] = {}
        self.intervention: dict[int, dict[str, float]] = {}
        self.mitm: dict[int, MITMConfig] = {}

        # Filled in by `bind`; every one of these depends on state that only
        # exists once the Simulator has assigned port offsets.
        self.gra_origin: GRA
        self.sysids: list[int] = []
        self.n_entities = 0
        self.rid_in_socks: dict[int, zmq.Socket[bytes]] = {}
        self.rid_out_socks: dict[int, zmq.Socket[bytes]] = {}
        self.done_sock: zmq.Socket[bytes]
        self.rid_in_threads: dict[int, threading.Thread] = {}
        self.rid_out_threads: dict[int, threading.Thread] = {}
        self.done_thread: threading.Thread
        self.stop_sys: dict[int, threading.Event] = {}
        self.stop_gcs: dict[str, threading.Event] = {}
        self.rid_locks: dict[int, threading.Lock] = {}

    def add_vehicle(self, vehicle: SimVehicle) -> None:
        """
        Add a vehicle to the scenario, registering every GCS monitoring it.

        A vehicle may be monitored by zero, one or several GCSs; any of them not
        registered yet is added along with it. Raises if the vehicle is already
        in the scenario.
        """
        if vehicle.sysid in self.vehicles:
            raise ValueError(
                f"Vehicle {vehicle.sysid} is already in the scenario; "
                "each sysid can only be added once."
            )
        self.vehicles[vehicle.sysid] = vehicle
        for gcs in list(vehicle.gcss):
            self._register_gcs(gcs)
            gcs.add_vehicle(vehicle)  # idempotent: keeps both sides in sync

    def add_gcs(self, gcs: SimGCS) -> None:
        """
        Add a GCS to the scenario together with the vehicles it monitors.

        The vehicles are the ones already linked to the GCS (through
        `SimGCS(vehicles=...)`, `SimGCS.add_vehicle` or `SimVehicle.assign_gcs`).
        One already in the scenario is left as is, so it simply ends up
        monitored by several GCSs; a new one is added. Raises if a GCS of this
        name is already registered.
        """
        if gcs.name in self.gcss:
            raise ValueError(
                f"A GCS named '{gcs.name}' is already in the scenario; "
                "GCS names must be unique."
            )
        self.gcss[gcs.name] = gcs
        for vehicle in list(gcs.vehicles):
            registered = self.vehicles.get(vehicle.sysid)
            if registered is None:
                self.add_vehicle(vehicle)
            elif registered is not vehicle:
                raise ValueError(
                    f"A different vehicle with sysid {vehicle.sysid} is "
                    "already in the scenario."
                )

    def _register_gcs(self, gcs: SimGCS) -> None:
        """Record a GCS, rejecting a second distinct instance of the same name."""
        registered = self.gcss.get(gcs.name)
        if registered is None:
            self.gcss[gcs.name] = gcs
        elif registered is not gcs:
            raise ValueError(
                f"A different GCS instance named '{gcs.name}' is "
                "already registered; share one SimGCS across its vehicles."
            )

    def bind(self, gra_origin: GRAPose, *, port_offset: int) -> None:
        """
        Attach this Oracle to a launched simulation.

        Opens the ZMQ sockets and creates the worker threads, so it must run
        after the Simulator has assigned every vehicle's port offset.
        `Simulator.launch()` calls this; you rarely need to.
        """
        if self._bound:
            raise RuntimeError(
                "This Oracle is already bound to a simulation; build a new one "
                "for a second run."
            )
        self.gra_origin = gra_origin.unpose()
        self.sysids = list(self.vehicles.keys())
        veh_port_offsets = {
            sysid: veh.port_offset_required for sysid, veh in self.vehicles.items()
        }
        self.n_entities = len(self.sysids) + len(self.gcss)

        # Sockets
        self._zmq_ctx = zmq.Context()
        self.rid_in_socks = create_zmq_sockets(
            self._zmq_ctx, VehPort.RID_UP, zmq.SUB, veh_port_offsets
        )
        self.rid_out_socks = create_zmq_sockets(
            self._zmq_ctx, VehPort.RID_DOWN, zmq.PUB, veh_port_offsets
        )
        self.done_sock = create_zmq_socket(
            self._zmq_ctx, zmq.ROUTER, SimPort.ORC_DONE, offset=port_offset
        )
        _active.add(self)

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
        self.stop_gcs = {gcs_name: threading.Event() for gcs_name in self.gcss}
        self.rid_locks = {sysid: threading.Lock() for sysid in self.sysids}
        self._bound = True

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
        if not self._bound:
            raise RuntimeError(
                "Oracle.run() called before the Oracle was bound to a "
                "simulation; call Simulator.launch() first."
            )
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

    def close(self) -> None:
        """Close all ZMQ sockets and terminate the context."""
        if not self._bound:
            return  # never opened anything
        self._zmq_ctx.destroy(linger=0)
        _active.discard(self)

    def update_rid(self, sysid: int):
        """Receive Remote ID messages from one Vehicle and update the store."""
        while not self.stop_sys[sysid].is_set():
            try:
                rid: RIDData = self.rid_in_socks[sysid].recv_pyobj()
                # Before the EKF converges the vehicle reports lat/lon 0,0,
                # which converts to an ENU point ~3000 km away and would set the
                # scale of any plot. Those are "no fix yet", not positions.
                has_fix = not (rid.gra_pos.lat == 0.0 and rid.gra_pos.lon == 0.0)
                if self.record_positions and has_fix:
                    self.paths.setdefault(sysid, []).append(rid.enu_pos)
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
                            f"{o_sysid},{round(o_pos.x, 3)},{round(o_pos.y, 3)},"
                            f"{round(o_pos.z, 3)},{round(o_spd, 3)},"
                            f"{round(o_cog, 3)},{round(o_ele, 3)}"
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
                        if (
                            ("Serial Number" in res)
                            and (str(o_sysid) in res["Serial Number"])
                            and (
                                str(sysid)
                                in res["Serial Number"][str(o_sysid)]["values"]
                            )
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

    def plot_trajectories(
        self,
        *,
        oracle: bool = True,
        gcss: bool = False,
        sysids: Iterable[int] | None = None,
        save: str | Path | None = None,
        show: bool = True,
    ) -> Figure | None:
        """
        Plot recorded trajectories in the local ENU frame.

        Two different views are available, and they are worth comparing:

        * `oracle` — this Oracle's own Remote ID track of **every** vehicle,
          including any that no GCS monitors. Requires `record_positions`.
        * `gcss` — what each GCS recorded from its telemetry stream, so only its
          own vehicles, and only where `SimGCS.record_positions` was on. This is
          the ground-side view: a MITM that drops telemetry shows up as a gap
          here while the Oracle track stays complete.

        `sysids` restricts the plot to those vehicles. Returns the figure, or
        `None` when there is nothing recorded to draw.
        """
        wanted = set(sysids) if sysids is not None else None
        series: list[tuple[str, ENUs, str, str]] = []

        if oracle:
            for sysid, track in sorted(self.paths.items()):
                if (wanted is None or sysid in wanted) and track:
                    series.append((f"Vehicle {sysid}", track, self._color(sysid), "o"))

        if gcss:
            for name in sorted(self.gcss):
                file = DATA_PATH / f"trajectories_{name}.pkl"
                if not file.exists():
                    logging.warning(
                        f"GCS {name} recorded no trajectories "
                        f"(record_positions off, or it never ran)"
                    )
                    continue
                with file.open("rb") as f:
                    recorded: dict[int, GRAs] = pickle.load(f)
                for sysid, gra_track in sorted(recorded.items()):
                    if wanted is not None and sysid not in wanted:
                        continue
                    if gra_track:
                        series.append((
                            f"Vehicle {sysid} · GCS {name}",
                            self.gra_origin.to_rel_all(gra_track),
                            self._color(sysid),
                            "^",
                        ))

        if not series:
            logging.warning("Nothing to plot: no trajectories were recorded")
            return None

        fig = plt.figure(figsize=(8, 8))  # type: ignore
        ax = fig.add_subplot(projection="3d", proj_type="ortho")  # type: ignore
        ax.set_title("ENU Trajectories")  # type: ignore
        ax.set_xlabel("East (m)")  # type: ignore
        ax.set_ylabel("North (m)")  # type: ignore
        ax.set_zlabel("Up (m)")  # type: ignore
        for label, track, color, marker in series:
            ax.scatter(  # type: ignore
                [p.x for p in track], [p.y for p in track], [p.z for p in track],
                c=[color], s=12, alpha=0.8, marker=marker, label=label,
                depthshade=True,
            )
        self._set_axes(ax, series)
        ax.legend(loc="best", fontsize=8)  # type: ignore
        plt.tight_layout()
        if save is not None:
            fig.savefig(save, dpi=150)  # type: ignore
            logging.info(f"Trajectory plot saved to '{save}'")
        if show:
            plt.show()  # type: ignore
        return fig

    @staticmethod
    def _set_axes(ax: Axes3D, series: list[tuple[str, ENUs, str, str]]) -> None:
        """
        Scale the axes so the track stays readable whatever its shape.

        * **East/North share one scale**, so the ground track is undistorted —
          a square circuit looks square. Both get the larger of the two extents,
          floored at `MIN_PLOT_SPAN` so a straight-line flight (which barely
          moves on one axis) does not collapse the view into a flat sheet.
        * **Up gets its own scale.** Altitudes are far smaller than ground
          distances, and tying them together buries the climb in empty space.
          It starts at the ground plane, never below, unless the track really
          goes there.
        """
        xs = [p.x for _, track, _, _ in series for p in track]
        ys = [p.y for _, track, _, _ in series for p in track]
        zs = [p.z for _, track, _, _ in series for p in track]

        ground = max(max(xs) - min(xs), max(ys) - min(ys), MIN_PLOT_SPAN)
        half = ground / 2 * 1.05  # margin so points are not on the edge
        for set_lim, vals in ((ax.set_xlim, xs), (ax.set_ylim, ys)):
            mid = (max(vals) + min(vals)) / 2
            set_lim(mid - half, mid + half)  # type: ignore

        # The floor is strict and never padded: it sits on the ground plane, or
        # exactly on the lowest sample when the track dips below it. Ground noise
        # puts a landed vehicle a few centimetres under zero, and that stays
        # visible at the bottom of the axis rather than being clipped away.
        low = min(0.0, min(zs))
        high = max(max(zs), low + MIN_UP_SPAN)
        ax.set_zlim(low, high + (high - low) * 0.05)  # type: ignore
        ax.set_box_aspect((1, 1, 0.6))  # type: ignore

    def _color(self, sysid: int) -> str:
        """Plot colour for a vehicle, taken from its own `SimVehicle.color`."""
        veh = self.vehicles.get(sysid)
        return str(veh.color.value) if veh is not None else str(Color.BLACK.value)
