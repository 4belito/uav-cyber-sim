"""Remote ID helper class."""

from __future__ import annotations

import copy
import logging
import math
import pickle
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq
from pymavlink import mavutil

from config import BasePort
from helpers.coordinates import ENU, GRA

from .connections.zeromq import create_zmq_socket

mav = mavutil.mavlink.MAVLink(None)

with open("fake_position.pkl", "rb") as f:
    fake_pos = pickle.load(f)

# fake_pos = ENU(0, 0, 0)  # East, North, Up in meters


def parse_one(buf: bytes) -> mavlink.MAVLink_gps_raw_int_message:
    """Parse a single MAVLink message from a byte buffer."""
    msg: mavlink.MAVLink_gps_raw_int_message = mav.parse_buffer(buf)[0]  # type: ignore
    try:
        return msg
    except Exception as e:
        raise ValueError(f"Failed to parse MAVLink message: {e}")


@dataclass
class RIDData:
    """Data class for Remote ID information."""

    sysid: int
    gra_pos: GRA  # global position
    enu_pos: ENU  # m/s relative to origin
    enu_vel: ENU  # m/s relative to uav
    speed: float  # m/s
    cog: float  # angle of course over ground,(0° = North, 90° = East)
    ele: float  # elevation angle,(0° = North, 90° = Up)
    rel_alt: float  # meters relative to takeoff
    hdg: float  # degrees - like cog but for uav heading
    last_update: float  # optional, handy for freshness checks

    # @classmethod
    # def none(cls, sysid: int) -> Self:
    #     """Create a RIDData instance with all fields set to None except sysid."""
    #     return cls(sysid=sysid)


class RIDManager:
    """Owns Remote ID state, ZMQ sockets, and background threads for one UAV."""

    def __init__(self, sysid: int, port_offset: int, gra_origin: GRA) -> None:
        self.gra_origin = gra_origin
        self.sysid = sysid
        self.data: RIDData
        self.received_rid: Queue[RIDData] = Queue()
        self._lock = threading.Lock()  # ???
        self._stop = threading.Event()
        self.pending = False  # whether there is new data to publish

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
        vx_cm = cast(int, payload.get("vx"))
        vy_cm = cast(int, payload.get("vy"))
        vz_cm = cast(int, payload.get("vz"))
        rel_alt = cast(int, payload.get("relative_alt"))
        hdg_centdegree = cast(int, payload.get("hdg"))
        gra_pos = GRA.from_global_int(lat_int, lon_int, alt_int)
        enu_pos = self.gra_origin.to_rel(gra_pos)
        enu_vel = ENU.from_ned(vx_cm / 100, vy_cm / 100, vz_cm / 100)
        ve, vn, vu = enu_vel
        cog = (math.degrees(math.atan2(ve, vn)) + 360) % 360
        ele = (math.degrees(math.atan2(vu, vn)) + 360) % 360
        speed = math.sqrt(ve**2 + vn**2 + vu**2)
        hdg = hdg_centdegree / 100.0
        with self._lock:
            self.data = RIDData(
                sysid=self.sysid,
                gra_pos=gra_pos,
                enu_pos=enu_pos,
                enu_vel=enu_vel,
                speed=speed,
                cog=cog,
                ele=ele,
                rel_alt=rel_alt,
                hdg=hdg,
                last_update=time.time(),
            )
            self.pending = True

    def publish(self) -> None:
        """Send current RID snapshot (pyobj) to oracle."""
        with self._lock:
            if self.pending:
                if self.sysid == 255:
                    send_data = copy.copy(self.data)
                    send_data.enu_pos = fake_pos
                else:
                    send_data = self.data
                logging.debug(f"SEND DATA RID({self.sysid}): {send_data}")
                self._out_sock.send_pyobj(send_data)  # type: ignore
                self.pending = False

    # --- background loops ------------------------------------------------------

    def _collect(self, sock: zmq.Socket[bytes]) -> None:
        """Collect RAW data from from the rid_data socket."""
        while not self._stop.is_set():
            try:
                buf = sock.recv()
                msg = parse_one(buf)
                # if msg and msg.get_type() == "GPS_RAW_INT" and msg.fix_type > 2:
                if (
                    msg
                    and msg.get_type() == "GLOBAL_POSITION_INT"
                    and (msg.lat != 0 or msg.lon != 0)
                ):
                    #logging.debug(f"RID({self.sysid}) collect: {msg.to_dict()}")
                    self.update(msg.to_dict())
            except zmq.Again:
                continue
            except Exception as e:
                logging.debug(f"RID collector noise: {e}")

    def _receive(self, sock: zmq.Socket[bytes]) -> None:
        """Receive the RID data retransmitted  from near uavs."""
        logging.debug(f"receive RID({self.sysid}) starting...")
        while not self._stop.is_set():
            logging.debug(f"Uav {self.sysid} waiting for RID data...")
            try:
                rid: RIDData = sock.recv_pyobj()  # type: ignore
                self.received_rid.put(rid)
                logging.debug(f"Uav {self.sysid} received RID: {rid.sysid}")
            except zmq.Again:
                logging.debug(f"Uav {self.sysid} no data available")
                continue
            except Exception as e:
                logging.error(f"RID receiver error: {e}")
