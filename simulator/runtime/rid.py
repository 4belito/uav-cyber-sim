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
from simulator.helpers.coordinates import ENU, GRA
from simulator.runtime.vehicle.adsb_conversion import rid_to_adsb_beacon


class RIDManager:
    """Owns Remote ID state, ZMQ sockets, and background threads for one UAV."""

    def __init__(self, sysid: int, port_offset: int, gra_origin: GRA) -> None:
        self.gra_origin = gra_origin
        self.sysid = sysid
        self.data: RIDData | None = None
        self.received_rid: Queue[RIDData] = Queue()
        self._lock = threading.Lock()  # ???
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

        self._threads: list[threading.Thread] = []

    # --- lifecycle -------------------------------------------------------------

    def start(self) -> None:
        """Start background collectors."""
        self._threads = [
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
        self._adsb_out_sock.close(linger=0)
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
            if self.pending and self.data:
                if self.fake_pos and self.sysid == 255:
                    send_data = copy.copy(self.data)
                    send_data.enu_pos = self.fake_pos
                    logging.debug(f"SEND FAKE DATA RID({self.sysid}): {send_data}")
                else:
                    send_data = self.data
                    logging.debug(f"SEND DATA RID({self.sysid}): {send_data}")

                self._out_sock.send_pyobj(send_data)  # type: ignore
                self.pending = False

    # --- background loops ------------------------------------------------------
    def _receive(self, sock: zmq.Socket[bytes]) -> None:
        """Receive the RID data retransmitted  from near uavs."""
        while not self._stop.is_set():
            try:
                rid: RIDData = sock.recv_pyobj()  # type: ignore
                self.received_rid.put(rid)
                logging.debug(f"Uav {self.sysid} received RID: {rid.sysid}")
                # sent to adsb manager or other logic as needed
                beacon = rid_to_adsb_beacon(rid)
                self._adsb_out_sock.send_pyobj(beacon)  # type: ignore
            except zmq.Again:
                continue
            except Exception as e:
                logging.error(f"RID receiver error: {e}")
