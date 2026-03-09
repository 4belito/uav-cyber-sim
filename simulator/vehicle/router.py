"""
Single MAVLink reader.
Reads from the connection and updates the shared vehicle state.
"""

from __future__ import annotations

import logging
import threading

from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.streams import decode_unknown_message
from simulator.vehicle.state import VehicleState


class MAVLinkRouter(threading.Thread):
    """
    Single MAVLink reader.
    Reads from the connection and updates the shared vehicle state.
    """

    def __init__(
        self,
        conn: MAVConnection,
        state: VehicleState,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.stop_event = stop_event

    def run(self) -> None:
        """Continuously read messages and update state until stopped."""
        while not self.stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue

                # Optional: decode UNKNOWN here if needed
                if msg.get_type().startswith("UNKNOWN"):
                    msg = decode_unknown_message(msg)

                self.state.update(msg)

            except Exception as exc:
                logging.error("Router failed: %s", exc)
