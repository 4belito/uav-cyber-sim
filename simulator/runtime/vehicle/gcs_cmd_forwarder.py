"""Forwards whitelisted MAVLink commands received from the GCS to ArduPilot SITL."""

from __future__ import annotations

import logging
import threading

from simulator.helpers.connections import MAVConnection

# Message types the GCS is permitted to send to the vehicle.
# Heartbeats and acknowledgements are excluded — Logic manages those independently.
GCS_FORWARD_TYPES: frozenset[str] = frozenset(
    {
        "MISSION_COUNT",
        "MISSION_ITEM_INT",
        "MISSION_ITEM",
        "MISSION_REQUEST_LIST",
        "MISSION_CLEAR_ALL",
        "MISSION_ACK",
        "COMMAND_LONG",
        "COMMAND_INT",
        "SET_MODE",
    }
)


class GCSCommandForwarder(threading.Thread):
    """Reads MAVLink messages arriving from the GCS and forwards whitelisted ones to SITL."""

    def __init__(self, src_conn: MAVConnection, dst_conn: MAVConnection) -> None:
        super().__init__(daemon=True)
        self.src_conn = src_conn
        self.dst_conn = dst_conn
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self.src_conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                if msg.get_type() in GCS_FORWARD_TYPES:
                    logging.info("GCS→SITL forwarding %s", msg.get_type())
                    self.dst_conn.mav.send(msg)
            except Exception as exc:
                logging.error("GCSCommandForwarder error: %s", exc)

    def stop(self) -> None:
        self._stop_event.set()
        self.join()
        self.src_conn.close()
