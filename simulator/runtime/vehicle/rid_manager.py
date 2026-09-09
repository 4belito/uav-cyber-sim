"""Remote ID helper class."""

from __future__ import annotations

import copy
import logging
import math
import threading
import time
from queue import Queue
from typing import cast

import zmq

from simulator.config import SimPort, VehPort
from simulator.entities.riddata import RIDData
from simulator.entities.spoof_profile import SpoofProfile
from simulator.helpers.connections import create_zmq_socket
from simulator.helpers.connections.mavlink.streams import make_json_safe
from simulator.helpers.coordinates import ENU, GRA
from simulator.helpers.logging.data_logger import DataLogger
from simulator.runtime.vehicle.adsb_conversion import rid_to_adsb_beacon


class RIDManager:
    """
    Manage Remote ID (RID) state, communication, and data collection for a single UAV.

    Responsibilities:
        - Maintain latest RID state derived from MAVLink telemetry
        - Publish RID data to the Oracle
        - Receive RID data from nearby UAVs
        - Forward received RID as ADS-B beacons
        - Record RID input/output streams for dataset generation
    """

    def __init__(
        self,
        sysid: int,
        port_offset: int,
        orc_port_offset: int,
        gra_origin: GRA,
        data_logger: DataLogger | None = None,
        spoof: SpoofProfile | None = None,
    ) -> None:
        self.gra_origin = gra_origin
        self.sysid = sysid
        self.data: RIDData | None = None
        self.received_rid: Queue[RIDData] = Queue()
        self._latest: dict[int, RIDData] = {}
        self._lock = threading.Lock()  # protects self.data and self.pending
        self._stop = threading.Event()
        self.pending = False  # whether there is new data to publish

        # RID spoofing: a `SpoofProfile` (or None to broadcast honestly). Times in
        # the profile are measured from this manager's start.
        self.spoof = spoof
        self._spoof_t0 = time.monotonic()

        # ZMQ setup
        self._ctx = zmq.Context()
        self._in_sock = create_zmq_socket(
            self._ctx, zmq.SUB, VehPort.RID_DOWN, port_offset
        )
        self._out_sock = create_zmq_socket(
            self._ctx, zmq.PUB, VehPort.RID_UP, port_offset
        )

        self._adsb_out_sock = create_zmq_socket(
            self._ctx,
            zmq.PUB,
            VehPort.ADSB,
            port_offset,
        )

        self._done_sock = create_zmq_socket(
            self._ctx,
            zmq.DEALER,
            SimPort.ORC_DONE,
            offset=orc_port_offset,
            timeout=-1,
            identity=f"log-{self.sysid}".encode(),
        )

        # Background threads
        self._threads: list[threading.Thread] = []

        # Dataset file (JSONL stream)
        self.logger = data_logger

    def start(self) -> None:
        """Start background threads for receiving RID data."""
        self._threads = [
            threading.Thread(target=self._receive, args=(self._in_sock,)),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        """Stop background threads and close all resources."""
        self._stop.set()

        for t in self._threads:
            t.join()

        self._done_sock.send_string("DONE")  # type: ignore
        self._wait_until_ack()
        self._in_sock.close(linger=0)
        self._out_sock.close(linger=0)
        self._adsb_out_sock.close(linger=0)
        self._done_sock.close(linger=0)
        self._ctx.term()

    def _wait_until_ack(self):
        """Wait until Oracle acknowledges DONE message."""
        while True:
            try:
                msg = self._done_sock.recv_string()
            except zmq.Again:
                continue

            if msg == "ACK":
                return
            else:
                logging.warning(f"RID {self.sysid} ignoring unexpected message: {msg}")

    # --- state update / publish -----------------------------------------------
    def update(self, payload: dict[str, str | float | int]) -> None:
        """Update internal RID state from MAVLink-derived payload."""
        rid = self._build_rid(payload)
        with self._lock:
            self.data = rid
            self.pending = True

    def publish(self) -> None:
        """Send current RID snapshot (pyobj) to oracle."""
        send_data: RIDData | None = None
        with self._lock:
            if self.pending and self.data is not None:
                send_data = self.data
                fake_pos = (
                    self.spoof.position_at(time.monotonic() - self._spoof_t0)
                    if self.spoof is not None
                    else None
                )
                if fake_pos is not None:
                    send_data = copy.copy(self.data)
                    send_data.enu_pos = fake_pos
                    # Neighbors convert RID -> ADS-B from gra_pos (lat/lon/alt),
                    # not enu_pos, so the geodetic position must be spoofed too or
                    # the victim keeps avoiding our true location.
                    send_data.gra_pos = self.gra_origin.to_abs(fake_pos)
                    logging.debug(f"SEND FAKE DATA RID({self.sysid}): {send_data}")
                else:
                    logging.debug(f"SEND DATA RID({self.sysid}): {send_data}")

                self._out_sock.send_pyobj(send_data)  # type: ignore
                self.pending = False
        if self.logger and send_data is not None:
            self.logger.write(
                {
                    "type": "rid_out",
                    "data": make_json_safe(send_data.to_dict()),
                }
            )

    # --- background loops ------------------------------------------------------
    def get_latest(self, sysid: int) -> RIDData | None:
        """Return the most recently received RID for a given sysid."""
        return self._latest.get(sysid)

    def _receive(self, sock: zmq.Socket[bytes]) -> None:
        """Continuously receive RID data from nearby UAVs."""
        while not self._stop.is_set():
            try:
                rid: RIDData = sock.recv_pyobj()  # type: ignore
                self._latest[rid.sysid] = rid
                self.received_rid.put(rid)
                logging.debug(f"Uav {self.sysid} received RID: {rid.sysid}")
                # Convert to ADS-B and forward
                beacon = rid_to_adsb_beacon(rid)
                self._adsb_out_sock.send_pyobj(beacon)  # type: ignore

                # Record incoming RID
                if self.logger:
                    self.logger.write(
                        {
                            "type": "rid_in",
                            "other_sysid": rid.sysid,
                            "data": make_json_safe(rid.to_dict()),
                        }
                    )
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as e:
                logging.error(f"RID receiver error: {e}")

    def _build_rid(self, payload: dict[str, str | float | int]) -> RIDData:
        lat = cast(int, payload.get("lat"))
        lon = cast(int, payload.get("lon"))
        alt = cast(int, payload.get("alt"))
        vx = cast(int, payload.get("vx")) / 100
        vy = cast(int, payload.get("vy")) / 100
        vz = cast(int, payload.get("vz")) / 100
        rel_alt = cast(int, payload.get("relative_alt"))
        hdg = cast(int, payload.get("hdg")) / 100.0

        gra = GRA.from_global_int(lat, lon, alt)
        enu_pos = self.gra_origin.to_rel(gra)
        enu_vel = ENU.from_ned(vx, vy, vz)

        ve, vn, vu = enu_vel

        speed = math.sqrt(ve**2 + vn**2 + vu**2)
        cog = (math.degrees(math.atan2(ve, vn)) + 360) % 360
        ele = (math.degrees(math.atan2(vu, vn)) + 360) % 360

        return RIDData(
            sysid=self.sysid,
            gra_pos=gra,
            enu_pos=enu_pos,
            enu_vel=enu_vel,
            speed=speed,
            cog=cog,
            ele=ele,
            rel_alt=rel_alt,
            hdg=hdg,
            last_update=time.time(),
        )
