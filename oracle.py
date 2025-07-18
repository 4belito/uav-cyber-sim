"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

import threading
from queue import Queue
from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink
import zmq

from config import BasePort
from helpers.change_coordinates import GRA
from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import MsgID
from mavlink.util import ask_msg
from monitor import UAVMonitor


class Oracle(UAVMonitor):
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self,
        conns: dict[int, MAVConnection],
        port_offsets: dict[int, int],
        name: str = "Oracle ⚪",
        verbose: int = 1,
    ) -> None:
        super().__init__(conns, name, verbose)
        self.rid_queue = Queue[tuple[int, bytes]]()
        self.zmq_ctx = zmq.Context()
        self.rid_in_socks = dict[int, zmq.Socket[bytes]]()
        self.rid_out_socks = dict[int, zmq.Socket[bytes]]()
        for sysid in conns.keys():
            self.rid_in_socks[sysid] = self.zmq_ctx.socket(zmq.SUB)
            self.rid_in_socks[sysid].connect(
                f"tcp://127.0.0.1:{BasePort.RID_UP + port_offsets[sysid]}"
            )
            self.rid_in_socks[sysid].setsockopt_string(zmq.SUBSCRIBE, "")
            self.rid_in_socks[sysid].setsockopt(zmq.RCVTIMEO, 100)
            self.rid_out_socks[sysid] = self.zmq_ctx.socket(zmq.PUB)
            self.rid_out_socks[sysid].bind(
                f"tcp://127.0.0.1:{BasePort.RID_DOWN + port_offsets[sysid]}"
            )

    def run(self):
        """Run the Oracle to manage UAV connections and communication."""
        rid_in_threads = [
            threading.Thread(target=self.enqueue_remote_ids, args=(sysid,))
            for sysid in self.conns.keys()
        ]
        rid_out_thread = threading.Thread(target=self.retransmit_remote_ids)

        if self.verbose:
            print(f"{self.name}: 🏁 Starting Oracle with {len(self.conns)} vehicles")

        for thread in rid_in_threads:
            thread.start()
        rid_out_thread.start()

        for conn in self.conns.values():
            ask_msg(
                conn, self.verbose, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000
            )

        while self.conns:
            for sysid, conn in list(self.conns.items()):
                try:
                    msg = conn.recv_msg()
                    if not msg:
                        continue
                except Exception:
                    continue
                match msg.get_type():
                    case "GLOBAL_POSITION_INT":
                        self._get_global_pos(
                            cast(mavlink.MAVLink_global_position_int_message, msg),
                            sysid,
                        )
                    case "STATUSTEXT":
                        msg = cast(mavlink.MAVLink_statustext_message, msg)
                        if self._is_plan_done(conn, msg, sysid):
                            self.remove(sysid)
                    case _:
                        pass

        for thread in rid_in_threads:
            thread.join()
        rid_out_thread.join()

        self.zmq_ctx.term()

    def enqueue_remote_ids(self, sysid: int):
        """Receive Remote ID messages from one UAV and add them to the queue."""
        while sysid in self.rid_in_socks:
            try:
                rid = self.rid_in_socks[sysid].recv()
            except Exception:
                continue
            self.rid_queue.put((sysid, rid))

    def retransmit_remote_ids(self):
        """Retransmit Remote IDs to other UAVs."""
        while self.rid_out_socks:
            try:
                sysid, rid = self.rid_queue.get(timeout=0.1)
            except Exception:
                continue
            if self.verbose > 1:
                print(f"{self.name}: 🔁 Received Remote ID from {sysid}")
            pos = self.pos.get(sysid, None)
            if pos is None:
                continue
            for other_sysid, other_sock in self.rid_out_socks.items():
                if other_sysid == sysid:
                    continue
                other_pos = self.pos.get(other_sysid, None)
                if other_pos is None:
                    continue
                dist = GRA.distance(pos, other_pos)
                if dist > 100:
                    continue
                other_sock.send(rid)  # type: ignore
                if self.verbose > 1:
                    print(
                        f"{self.name}: 🔁 Retransmitted Remote ID from {sysid} "
                        f"to {other_sysid}"
                    )

    def remove(self, sysid: int):
        """Remove a UAV connection and its associated sockets."""
        super().remove(sysid)
        del self.rid_in_socks[sysid]
        del self.rid_out_socks[sysid]
