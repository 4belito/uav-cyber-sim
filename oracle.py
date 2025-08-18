"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

import logging
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
        uav_port_offsets: dict[int, int],
        gcs_port_offsets: dict[str, int],
        gcs_sysids: dict[str, list[int]],
    ) -> None:
        super().__init__(conns)
        self.gcs_sysids = gcs_sysids
        self.rid_queue = Queue[tuple[int, bytes]]()
        self.zmq_ctx = zmq.Context()

        self.rid_in_socks = dict[int, zmq.Socket[bytes]]()
        self.rid_out_socks = dict[int, zmq.Socket[bytes]]()
        for sysid in conns.keys():
            self.rid_in_socks[sysid] = self.zmq_ctx.socket(zmq.SUB)
            self.rid_in_socks[sysid].connect(
                f"tcp://127.0.0.1:{BasePort.RID_UP + uav_port_offsets[sysid]}"
            )
            self.rid_in_socks[sysid].setsockopt_string(zmq.SUBSCRIBE, "")
            self.rid_in_socks[sysid].setsockopt(zmq.RCVTIMEO, 100)
            self.rid_out_socks[sysid] = self.zmq_ctx.socket(zmq.PUB)
            self.rid_out_socks[sysid].bind(
                f"tcp://127.0.0.1:{BasePort.RID_DOWN + uav_port_offsets[sysid]}"
            )
            self.rid_out_socks[sysid].setsockopt(zmq.SNDTIMEO, 100)

        self.gcs_socks = dict[str, zmq.Socket[bytes]]()
        for gcs_name in gcs_port_offsets.keys():
            self.gcs_socks[gcs_name] = self.zmq_ctx.socket(zmq.SUB)
            self.gcs_socks[gcs_name].connect(
                f"tcp://127.0.0.1:{BasePort.GCS_ZMQ + gcs_port_offsets[gcs_name]}"
            )
            self.gcs_socks[gcs_name].setsockopt_string(zmq.SUBSCRIBE, "")
            self.gcs_socks[gcs_name].setsockopt(zmq.RCVTIMEO, 100)

        # Small delay to ensure ZMQ connections are established
        import time

        time.sleep(0.2)

    def run(self):
        """Run the Oracle to manage UAV connections and communication."""
        rid_in_threads = [
            threading.Thread(target=self.enqueue_remote_ids, args=(sysid,))
            for sysid in self.conns.keys()
        ]
        rid_out_thread = threading.Thread(target=self.retransmit_remote_ids)

        logging.info(f"🏁 Starting Oracle with {len(self.conns)} vehicles")
        logging.info(f"Monitoring {len(self.gcs_socks)} GCS processes")

        for thread in rid_in_threads:
            thread.start()
        rid_out_thread.start()

        for conn in self.conns.values():
            ask_msg(conn, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000)

        logging.debug("Entering main monitoring loop...")

        while self.conns:
            logging.debug(
                f"Loop iteration - {len(self.conns)} UAV connections remaining"
            )

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
                    case _:
                        pass

            # Check for GCS completion messages
            logging.debug(f"Checking {len(self.gcs_socks)} GCS sockets for messages...")

            messages_received = 0
            for gcs_name, sock in list(self.gcs_socks.items()):
                try:
                    msg = sock.recv_string(flags=zmq.NOBLOCK)
                    messages_received += 1
                    logging.info(f"Received message '{msg}' from GCS {gcs_name}")
                    if msg == "DONE":
                        self.remove_gcs(gcs_name)
                    else:
                        logging.debug(f"Ignoring non-DONE message: {msg}")
                except zmq.Again:
                    # No message available, this is expected
                    continue
                except Exception as e:
                    logging.error(f"Error receiving from GCS {gcs_name}: {e}")
                    continue

            if messages_received == 0:
                logging.debug("No messages received from any GCS this iteration")
            else:
                logging.debug(f"Received {messages_received} messages this iteration")

        logging.info("✅ Main monitoring loop completed - all connections closed")

        for thread in rid_in_threads:
            thread.join()
        rid_out_thread.join()

        logging.info("🎉 Oracle shutdown complete!")

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
            logging.debug(f"🔁 Received Remote ID from {sysid}")
            pos = self.pos.get(sysid, None)
            if pos is None:
                continue
            for other_sysid, other_sock in list(self.rid_out_socks.items()):
                if other_sysid == sysid:
                    continue
                other_pos = self.pos.get(other_sysid, None)
                if other_pos is None:
                    continue
                dist = GRA.distance(pos, other_pos)
                if dist > 100:
                    continue
                try:
                    other_sock.send(rid)  # type: ignore
                    logging.debug(
                        f"🔁 Retransmitted Remote ID from {sysid} to {other_sysid}"
                    )
                except Exception:
                    pass

    def remove_gcs(self, gcs_name: str):
        """Remove a GCS connection and its associated sockets."""
        logging.debug(
            f"Removing GCS {gcs_name} and its UAVs: {self.gcs_sysids[gcs_name]}"
        )
        del self.gcs_socks[gcs_name]
        for sysid in self.gcs_sysids[gcs_name]:
            logging.debug(f"Removing UAV {sysid} from tracking")
            self.remove_uav(sysid)
        logging.info(
            f"GCS {gcs_name} removal complete. Remaining connections: {len(self.conns)}"
        )

    def remove_uav(self, sysid: int):
        """Remove a UAV connection and its associated sockets."""
        super().remove_uav(sysid)
        del self.rid_in_socks[sysid]
        del self.rid_out_socks[sysid]
