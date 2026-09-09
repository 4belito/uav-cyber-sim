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
import re
import subprocess
import threading
import time
import unicodedata
from collections.abc import Iterable
from pathlib import Path
from typing import Literal

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
from simulator.helpers.logging.log_reader import read_true_track
from simulator.runtime.grid import Grid

# Smallest ground span a plot is given, so a stationary or straight-line flight
# is not zoomed into centimetres of numerical noise.
MIN_PLOT_SPAN = 5.0  # metres
# The Up axis gets a much smaller floor: altitude detail is worth seeing, and a
# 5 m floor would bury a low hop or the centimetre-scale wobble of a landed
# vehicle. Only a genuinely flat track is padded to this.
MIN_UP_SPAN = 1.0  # metres
# One marker per GCS, so two stations watching the same vehicle stay apart on a
# plot: the colour says which vehicle, the marker says which recorded it.
# The Oracle's own Remote ID view always uses "o".
GCS_MARKERS = ("^", "s", "D", "v", "P", "X", "*", "<", ">", "h")
COLOR_EMOJI_LABELS = {
    "🟦": "BLUE",
    "🟩": "GREEN",
    "🟥": "RED",
    "🟧": "ORANGE",
    "🟨": "YELLOW",
    "⬛": "BLACK",
    "⬜": "WHITE",
}

TX_LOOP_SLEEP = 0.01
RX_LOOP_SLEEP = 0.10
# How long `stop()` waits on each worker thread. They only ever sleep for a
# loop tick, so anything longer means the thread is wedged and is not worth
# waiting for — the run is being torn down either way.
THREAD_JOIN_TIMEOUT = 2.0

# Module-level registry so clean() can reach active Oracle instances
_active: set["Oracle"] = set()


def _legend_safe(name: str) -> str:
    """
    Return a plot-font-safe GCS label without losing color identity.

    GCS names often carry a colour emoji (`GCS_🟩🟦`), and matplotlib's default
    font has no glyph for those: it warns once per character and draws a
    placeholder box. Replace supported color emoji with ASCII tags such as
    `GCS [GREEN] [BLUE]`, then drop any other unsupported characters. The GCS
    keeps its real name everywhere else, including its trajectory file.
    """
    for emoji, color_name in COLOR_EMOJI_LABELS.items():
        name = re.sub(rf"[_\-\s]*{re.escape(emoji)}", f" [{color_name}]", name)
    kept = "".join(
        ch for ch in name if ord(ch) <= 0xFFFF and unicodedata.category(ch) != "So"
    ).strip(" _-·")
    return kept or name


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to Vehicle logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self,
        transmission_range: float = 100.0,
        record_positions: bool = True,
        network_sim: bool = False,
        rid_frequency: int = 5,
        rid_enabled: bool = True,
    ) -> None:
        """
        Configure an Oracle. It is not usable until `bind` supplies the
        launch-time wiring, which `Simulator.launch()` does for you.

        Everything the Oracle itself decides is an argument here:

        * `rid_enabled` — relay Remote ID between vehicles at all. Turn it off
          and no vehicle ever hears another, so nothing avoids anything; the
          Oracle still receives each vehicle's own beacons, so `record_positions`
          and the plots keep working.
        * `transmission_range` — inter-vehicle Remote ID range, in metres.
        * `rid_frequency` — how often each vehicle broadcasts Remote ID, in Hz.
          The transmit half of the same model as `transmission_range`; the
          Simulator passes it to each vehicle's logic process.
        * `record_positions` — keep each vehicle's Remote ID track for
          `plot_trajectories`; covers every vehicle, GCS-monitored or not.
        * `network_sim` — relay Remote ID through the external `uli-net-sim` RF
          model instead of the plain range check. Needs that tool installed at
          `/usr/uli-net-sim`; the range check is used when it is off.

        Settings that cross a process boundary (heartbeat and stream rates, SITL
        speedup) are not here — they live in `simulator.params.simulation`,
        because the vehicle, GCS and SITL processes read them, not the Oracle.
        """
        self.transmission_range = transmission_range
        self.record_positions = record_positions
        self.network_sim = network_sim
        self.rid_frequency = rid_frequency
        self.rid_enabled = rid_enabled
        # sysid -> transmitted ENU track, filled from Remote ID as the run
        # proceeds. This is what each vehicle broadcast, so it is the *spoofed*
        # position for an attacker, not necessarily where the drone really was.
        self.paths: dict[int, ENUs] = {}
        # sysid -> real ENU track, reconstructed on demand from the ground-truth
        # logs (see `load_true_paths`). Empty until asked for.
        self.true_paths: dict[int, ENUs] = {}
        self.grid = Grid(cell_size=transmission_range * 1.01)
        self._seen_in_grid: set[int] = set()
        self._bound = False
        # Set by `stop()` to release the workers when a run is cut short.
        self._shutdown = threading.Event()

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
        monitored by several GCSs; a new one is added.

        Adding the same GCS twice is a no-op, because a GCS also arrives here
        indirectly: adding one GCS registers every other GCS of the vehicles it
        shares. Only a *different* GCS reusing a registered name is an error.
        """
        known = self.gcss.get(gcs.name)
        if known is not None and known is not gcs:
            raise ValueError(
                f"A different GCS named '{gcs.name}' is already in the "
                "scenario; GCS names must be unique."
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

        while len(seen_done) < self.n_entities and not self._shutdown.is_set():
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

    def run(self, timeout: float | None = None) -> bool:
        """
        Run the Oracle to manage Vehicle connections and communication.

        Blocks until every vehicle and every GCS has reported DONE. `timeout`
        caps that wait in seconds: a scenario that never finishes on its own —
        a pursuit with no capture, a vehicle stuck mid-plan — would otherwise
        block forever. The default `None` waits indefinitely, as before.

        Returns True when everything completed, False when the timeout ended
        the wait. Either way the Oracle's own threads are wound down before
        returning, so nothing is left spinning; the simulation's *processes*
        outlive this call, and `Simulator.stop()` is what ends those.
        """
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

        # One deadline for both waits, not one each: `timeout` is how long the
        # whole run may take, so vehicles finishing late leave the GCSs less.
        deadline = None if timeout is None else time.monotonic() + timeout
        completed = self._wait_for(self.stop_sys.values(), deadline)
        if completed:
            logging.info("✅ All Vehicle threads completed")
            completed = self._wait_for(self.stop_gcs.values(), deadline)
            if completed:
                logging.info("✅ All GCS threads completed")
        if not completed:
            pending_veh = [s for s, e in self.stop_sys.items() if not e.is_set()]
            pending_gcs = [n for n, e in self.stop_gcs.items() if not e.is_set()]
            logging.warning(
                f"⏱️ Timed out after {timeout}s — still running: "
                f"vehicles {pending_veh or 'none'}, GCSs {pending_gcs or 'none'}"
            )
        self.stop()

        logging.info("🎉 Oracle shutdown complete!")
        return completed

    @staticmethod
    def _wait_for(events: Iterable[threading.Event], deadline: float | None) -> bool:
        """Wait for every event, or until `deadline`. True if all were set."""
        pending = list(events)
        while any(not event.is_set() for event in pending):
            if deadline is not None and time.monotonic() >= deadline:
                return False
            time.sleep(0.1)
        return True

    def stop(self) -> None:
        """
        Wind down the Oracle's worker threads, finished or not.

        Releases every waiter — `wait_done` included, which otherwise sits on
        the DONE socket until each entity reports in — so a timed-out or
        interrupted run leaves no live thread behind in a notebook kernel.
        Idempotent, and safe on an Oracle that was never run. The sockets stay
        open so recorded state is still readable; `close()` frees those.
        """
        if not self._bound:
            return
        self._shutdown.set()
        for event in (*self.stop_sys.values(), *self.stop_gcs.values()):
            event.set()
        for thread in (
            *self.rid_in_threads.values(),
            *self.rid_out_threads.values(),
            self.done_thread,
        ):
            if thread.is_alive():
                thread.join(timeout=THREAD_JOIN_TIMEOUT)

    def close(self) -> None:
        """Close all ZMQ sockets and terminate the context."""
        if not self._bound:
            return  # never opened anything
        # Threads first: destroying the context under a thread still blocked in
        # `recv` is what leaves a kernel with wedged workers.
        self.stop()
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
        if not self.rid_enabled:
            # Nothing is relayed, so this thread has no work: vehicles still
            # report to the Oracle, they just never hear each other.
            logging.info(f"Remote ID relay disabled: vehicle {sysid} sends only")
            return
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
                if self.network_sim:
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
                        sysid, rid.enu_pos, radius=self.transmission_range
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
                        sysid, rid.enu_pos, radius=self.transmission_range
                    ):
                        with self.rid_locks[o_sysid]:
                            self.rid_out_socks[o_sysid].send_pyobj(rid)  # type: ignore
            except Exception as e:
                logging.error(f"Retransmit error for {sysid} of type {type(e)}: {e}")
            time.sleep(TX_LOOP_SLEEP)

    def load_true_paths(self) -> dict[int, ENUs]:
        """
        Reconstruct every vehicle's **real** trajectory from the ground-truth logs.

        The Oracle's live `self.paths` are the *transmitted* Remote ID positions,
        so an attacker appears at its spoofed location. The real flight is
        recorded independently as `GLOBAL_POSITION_INT` in each vehicle's MAVLink
        log; this reads those back into `self.true_paths` (keyed by sysid) and
        returns it. A vehicle whose log is missing or empty is skipped with a
        warning. Requires `bind` to have run (it needs `self.gra_origin`).
        """
        msgs_dir = DATA_PATH / "msgs"
        for sysid in sorted(self.vehicles):
            track = read_true_track(msgs_dir, sysid, self.gra_origin)
            if track:
                self.true_paths[sysid] = track
            else:
                logging.warning(
                    f"No ground-truth track for vehicle {sysid} "
                    f"(no log at {msgs_dir / f'veh_{sysid}.jsonl'}, or no fix)"
                )
        return self.true_paths

    def plot_trajectories(
        self,
        *,
        oracle: bool = True,
        truth: bool | Iterable[int] = False,
        gcss: Iterable[str] | Literal["all"] = (),
        sysids: Iterable[int] | None = None,
        legend: bool = True,
        xlim: tuple[float, float] | None = None,
        ylim: tuple[float, float] | None = None,
        zlim: tuple[float, float] | None = None,
        elev: float = 30.0,
        azim: float = -60.0,
        roll: float = 0.0,
        save: str | Path | None = None,
        show: bool = True,
    ) -> Figure | None:
        """
        Plot recorded trajectories in the local ENU frame.

        Two different views are available, and they are worth comparing:

        * `oracle` — this Oracle's own Remote ID track of **every** vehicle,
          including any that no GCS monitors. Requires `record_positions`. This
          is the *transmitted* view, so an attacker shows up at the position it
          spoofs, drawn as dots.
        * `truth` — the **real** trajectory reconstructed from the ground-truth
          logs (`load_true_paths`), drawn as a line so it reads apart from the
          transmitted dots. `False` (default) omits it; `True` overlays every
          vehicle; an iterable of sysids overlays only those (e.g. `[255]` for
          just the attacker). Overlaying makes an RID spoof obvious — the real
          line diverges from the spoofed dot — while an honest vehicle's line
          sits on its own dots. When the overlay is on, the two are tagged
          `(RID)` and `(real)` in the legend.
        * `gcss` — what the named GCSs recorded from their telemetry streams, so
          only their own vehicles, and only where `SimGCS.record_positions` was
          on. This is the ground-side view: a MITM that drops telemetry shows up
          as a gap here while the Oracle track stays complete. Stations are
          chosen by name: `"all"` for every one, a list of names (or a single
          name) for some, or the default empty for none. Each gets its own
          marker, so two stations watching one vehicle stay apart.

        `sysids` restricts the plot to those vehicles. Set `legend=False` to drop
        the key, which crowds the figure once there are many vehicles. Each of
        `xlim`, `ylim` and `zlim` takes a `(low, high)` pair in metres that
        replaces the automatic range on that axis, so separate runs can be
        compared on identical axes; the others still scale to fit.

        `elev`, `azim` and `roll` rotate the camera, in degrees: `elev` above the
        east/north plane, `azim` around the up axis (counter-clockwise, so `-90`
        looks along north and `0` along east), and `roll` about the line of
        sight. They default to matplotlib's own view (30, -60, 0). `elev=90,
        azim=-90` gives a plain top-down ground track.

        Returns the figure, or `None` when there is nothing recorded to draw.
        """
        wanted = set(sysids) if sysids is not None else None
        series: list[tuple[str, ENUs, str, str]] = []

        # `truth` selects which vehicles get a real-trajectory overlay: none,
        # all (`True`), or an explicit set of sysids.
        if truth is True:
            truth_wanted: set[int] | None = None  # every vehicle
        elif truth is False:
            truth_wanted = set()  # no overlay
        else:
            truth_wanted = set(truth)
        overlay = truth is True or bool(truth_wanted)
        # Only distinguish the two in the legend when both are actually shown;
        # a plain call keeps its original labels.
        rid_suffix = " (RID)" if overlay else ""

        if oracle:
            for sysid, track in sorted(self.paths.items()):
                if (wanted is None or sysid in wanted) and track:
                    series.append(
                        (
                            f"Vehicle {sysid}{rid_suffix}",
                            track,
                            self._color(sysid),
                            "o",
                        )
                    )

        truth_series: list[tuple[str, ENUs, str]] = []
        if overlay:
            if not self.true_paths:
                self.load_true_paths()
            for sysid, track in sorted(self.true_paths.items()):
                in_view = wanted is None or sysid in wanted
                in_truth = truth_wanted is None or sysid in truth_wanted
                if in_view and in_truth and track:
                    truth_series.append(
                        (f"Vehicle {sysid} (real)", track, self._color(sysid))
                    )

        # Marker is fixed by the station's position in the scenario, not by
        # what this call asked for, so a GCS keeps the same symbol between plots.
        marker_of = {
            name: GCS_MARKERS[i % len(GCS_MARKERS)]
            for i, name in enumerate(sorted(self.gcss))
        }
        if gcss == "all":
            names = sorted(self.gcss)
        else:
            # A bare string is a single name, not a sequence of characters.
            names = [gcss] if isinstance(gcss, str) else list(gcss)
        for name in names:
            if name not in self.gcss:
                logging.warning(f"No GCS named '{name}' in this scenario")
                continue
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
                if (wanted is None or sysid in wanted) and gra_track:
                    # Stations are often named "GCS_..." already; do not say it twice.
                    shown = _legend_safe(name)
                    prefix = "" if shown.upper().startswith("GCS") else "GCS "
                    series.append(
                        (
                            f"Vehicle {sysid} · {prefix}{shown}",
                            self.gra_origin.to_rel_all(gra_track),
                            self._color(sysid),
                            marker_of[name],
                        )
                    )

        if not series and not truth_series:
            logging.warning("Nothing to plot: no trajectories were recorded")
            return None

        fig = plt.figure(figsize=(8, 8))  # type: ignore
        ax = fig.add_subplot(projection="3d", proj_type="ortho")  # type: ignore
        # Defaults match matplotlib's own view (30, -60, 0), so the untouched
        # call renders the same as before these were made explicit.
        ax.view_init(elev=elev, azim=azim, roll=roll)  # type: ignore
        ax.set_title("ENU Trajectories")  # type: ignore
        ax.set_xlabel("East (m)")  # type: ignore
        ax.set_ylabel("North (m)")  # type: ignore
        ax.set_zlabel("Up (m)")  # type: ignore
        for label, track, color, marker in series:
            ax.scatter(  # type: ignore
                [p.x for p in track],
                [p.y for p in track],
                [p.z for p in track],  # type: ignore
                c=[color],
                s=12,
                alpha=0.8,
                marker=marker,
                label=label,
                depthshade=True,
            )
        # Real trajectories are drawn as lines so they read as a continuous path
        # against the transmitted dots, even sharing a vehicle's colour.
        for label, track, color in truth_series:
            ax.plot(  # type: ignore
                [p.x for p in track],
                [p.y for p in track],
                [p.z for p in track],
                color=color,
                linewidth=1.5,
                alpha=0.9,
                label=label,
            )
        # `_set_axes` only reads track points, so the marker slot is a filler.
        axis_series = series + [(lbl, trk, col, "o") for lbl, trk, col in truth_series]
        self._set_axes(ax, axis_series, xlim=xlim, ylim=ylim, zlim=zlim)
        if legend:
            ax.legend(loc="best", fontsize=8)  # type: ignore
        plt.tight_layout()
        if save is not None:
            fig.savefig(save, dpi=150)  # type: ignore
            logging.info(f"Trajectory plot saved to '{save}'")
        if show:
            plt.show()  # type: ignore
        return fig

    @staticmethod
    def _set_axes(
        ax: Axes3D,
        series: list[tuple[str, ENUs, str, str]],
        *,
        xlim: tuple[float, float] | None = None,
        ylim: tuple[float, float] | None = None,
        zlim: tuple[float, float] | None = None,
    ) -> None:
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

        An explicit `xlim`/`ylim`/`zlim` overrides the computed range on that
        axis and is used exactly as given, margins included.
        """
        xs = [p.x for _, track, _, _ in series for p in track]
        ys = [p.y for _, track, _, _ in series for p in track]
        zs = [p.z for _, track, _, _ in series for p in track]

        ground = max(max(xs) - min(xs), max(ys) - min(ys), MIN_PLOT_SPAN)
        half = ground / 2 * 1.05  # margin so points are not on the edge
        for set_lim, vals, lim in (  # type: ignore
            (ax.set_xlim, xs, xlim),  # type: ignore
            (ax.set_ylim, ys, ylim),  # type: ignore
        ):
            if lim is not None:
                set_lim(*lim)  # type: ignore
                continue
            mid = (max(vals) + min(vals)) / 2
            set_lim(mid - half, mid + half)  # type: ignore

        # The floor is strict and never padded: it sits on the ground plane, or
        # exactly on the lowest sample when the track dips below it. Ground noise
        # puts a landed vehicle a few centimetres under zero, and that stays
        # visible at the bottom of the axis rather than being clipped away.
        if zlim is not None:
            ax.set_zlim(*zlim)  # type: ignore
        else:
            low = min(0.0, min(zs))
            high = max(max(zs), low + MIN_UP_SPAN)
            ax.set_zlim(low, high + (high - low) * 0.05)  # type: ignore
        ax.set_box_aspect((1, 1, 0.6))  # type: ignore

    def _color(self, sysid: int) -> str:
        """Plot colour for a vehicle, taken from its own `SimVehicle.color`."""
        veh = self.vehicles.get(sysid)
        return str(veh.color.value) if veh is not None else str(Color.BLACK.value)
