"""Remote ID helper class."""

from __future__ import annotations

import copy
import logging
import math
import pickle
import threading
import time
from queue import Queue
from typing import cast

import zmq

from simulator.config import DATA_PATH, BasePort
from simulator.entities.riddata import RIDData
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
        gra_origin: GRA,
        data_logger: DataLogger | None = None,
    ) -> None:
        self.gra_origin = gra_origin
        self.sysid = sysid
        self.data: RIDData | None = None
        self.received_rid: Queue[RIDData] = Queue()
        self._lock = threading.Lock()  # protects self.data and self.pending
        self._stop = threading.Event()
        self.pending = False  # whether there is new data to publish

        # TODO: load fake position from config
        fake_pos_path = DATA_PATH / "fake_position.pkl"
        if fake_pos_path.exists():
            with open(fake_pos_path, "rb") as f:
                self.fake_pos = pickle.load(f)
        else:
            self.fake_pos = None

        # ZMQ setup
        self._ctx = zmq.Context()
        self._in_sock = create_zmq_socket(
            self._ctx, zmq.SUB, BasePort.RID_DOWN, port_offset
        )
        self._out_sock = create_zmq_socket(
            self._ctx, zmq.PUB, BasePort.RID_UP, port_offset
        )

        self._adsb_out_sock = create_zmq_socket(
            self._ctx,
            zmq.PUB,
            BasePort.ADSB_DOWN,
            port_offset,
        )

        # Background threads
        self._threads: list[threading.Thread] = []

        # Dataset file (JSONL stream)
        self.logger = data_logger

    def start(self) -> None:
        """Start background threads for receiving RID data."""
        self._threads = [
            threading.Thread(target=self._receive, args=(self._in_sock,), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        """Stop background threads and close all resources."""
        self._stop.set()
        for t in self._threads:
            t.join()
        self._in_sock.close(linger=0)
        self._out_sock.send_pyobj("DONE")  # type: ignore
        self._out_sock.close(linger=0)
        self._adsb_out_sock.close(linger=0)
        self._ctx.term()

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
                if self.fake_pos and self.sysid == 255:
                    send_data = copy.copy(self.data)
                    send_data.enu_pos = self.fake_pos
                    logging.debug(f"SEND FAKE DATA RID({self.sysid}): {send_data}")
                else:
                    send_data = self.data
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
    def _receive(self, sock: zmq.Socket[bytes]) -> None:
        """Continuously receive RID data from nearby UAVs."""
        while not self._stop.is_set():
            try:
                rid: RIDData = sock.recv_pyobj()  # type: ignore
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
