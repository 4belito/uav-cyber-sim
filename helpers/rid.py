"""Remote ID helper class."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Self, cast

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

from config import BasePort
from helpers.coordinates import GRA

from .connections.zeromq import create_zmq_socket

mav = mavutil.mavlink.MAVLink(None)


def parse_one(buf: bytes) -> mavlink.MAVLink_gps_raw_int_message:
    """Parse a single MAVLink message from a byte buffer."""
    msgs: list[mavlink.MAVLink_gps_raw_int_message] = mav.parse_buffer(buf)  # type: ignore
    try:
        return msgs[0]
    except Exception as e:
        raise ValueError(f"Failed to parse MAVLink message: {e}")


@dataclass
class RIDData:
    """Data class for Remote ID information."""

    sysid: int
    lat: float | None = None
    lon: float | None = None
    alt: float | None = None
    vel: float | None = None
    cog: float | None = None
    last_update: float | None = None  # optional, handy for freshness checks

    @classmethod
    def none(cls, sysid: int) -> Self:
        """Create a RIDData instance with all fields set to None except sysid."""
        return cls(sysid=sysid)


class RIDManager:
    """Owns Remote ID state, ZMQ sockets, and background threads for one UAV."""

    def __init__(self, sysid: int, port_offset: int):
        self.sysid = sysid
        self.data = RIDData(sysid=sysid)
        self._lock = threading.Lock()  # ???
        self._stop = threading.Event()

        # ZMQ setup
        self._ctx = zmq.Context()
        self._in_sock = create_zmq_socket(
            self._ctx, zmq.SUB, BasePort.RID_DOWN, port_offset
        )
        self._out_sock = create_zmq_socket(
            self._ctx, zmq.PUB, BasePort.RID_UP, port_offset
        )
        self._data_sock = create_zmq_socket(
            self._ctx, zmq.SUB, BasePort.RID_DATA, port_offset
        )

        self._threads: list[threading.Thread] = []

    # --- lifecycle -------------------------------------------------------------

    def start(self) -> None:
        """Start background collectors."""
        self._threads = [
            threading.Thread(
                target=self._collect, args=(self._data_sock,), daemon=True
            ),
            threading.Thread(target=self._receive, args=(self._in_sock,), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        """Stop threads and close sockets."""
        self._stop.set()
        for t in self._threads:
            t.join()
        self._in_sock.close(linger=0)
        self._out_sock.send_pyobj("DONE")  # type: ignore
        self._out_sock.close(linger=0)
        self._data_sock.close(linger=0)
        self._ctx.term()

    # --- state update / publish -----------------------------------------------

    def update(self, payload: dict[str, str | float | int]) -> None:
        """Atomic snapshot policy: overwrite with None when a key is missing."""
        lat_int = cast(int, payload.get("lat"))
        lon_int = cast(int, payload.get("lon"))
        alt_int = cast(int, payload.get("alt"))
        lat, lon, alt = GRA.from_global_int(lat_int, lon_int, alt_int)
        with self._lock:
            self.data.lat = lat
            self.data.lon = lon
            self.data.alt = alt
            self.data.vel = cast(float, payload.get("vel"))
            self.data.cog = cast(float, payload.get("cog"))
            self.data.last_update = time.time()

    def publish(self) -> None:
        """Send current RID snapshot (pyobj) to oracle."""
        with self._lock:
            if self.data.last_update:
                self._out_sock.send_pyobj(self.data)  # type: ignore

    # --- background loops ------------------------------------------------------

    def _collect(self, sock: zmq.Socket[bytes]) -> None:
        """Collect RAW data from from the rid_data socket."""
        while not self._stop.is_set():
            try:
                buf = sock.recv()
                msg = parse_one(buf)
                if msg and msg.get_type() == "GPS_RAW_INT" and msg.fix_type > 2:
                    logging.debug(f"RID({self.sysid}) collect: {msg.to_dict()}")
                    self.update(msg.to_dict())
            except zmq.Again:
                continue
            except Exception as e:
                logging.debug(f"RID collector noise: {e}")

    def _receive(self, sock: zmq.Socket[bytes]) -> None:
        """Receive the RID data retransmitted  from near uavs."""
        logging.debug(f"receive RID({self.sysid}) starting...")
        while not self._stop.is_set():
            try:
                rid: RIDData = sock.recv_pyobj()  # type: ignore
                logging.debug(f"Uav {self.sysid} received RID: {rid}")
            except zmq.Again:
                continue
            except Exception as e:
                logging.error(f"RID receiver error: {e}")
