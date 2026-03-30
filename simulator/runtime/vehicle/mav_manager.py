"""
Single MAVLink reader.
Reads from the connection and updates the shared vehicle state.
"""

from __future__ import annotations

import logging
import threading
import time

import pymavlink.dialects.v20.ardupilotmega as mavlink

from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)
from simulator.helpers.connections.mavlink.streams import (
    decode_unknown_message,
    make_json_safe,
)
from simulator.helpers.logging.data_logger import DataLogger


class MAVLinkManager(threading.Thread):
    """
    MAVLink I/O manager.

    - Receives MAVLink messages (RX)
    - Sends MAVLink messages (TX)
    - Logs all traffic
    """

    def __init__(
        self,
        conn: MAVConnection,
        state: VehicleStateP,
        stop_event: threading.Event,
        data_logger: DataLogger | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.stop_event = stop_event
        self.data_logger = data_logger

    def run(self) -> None:
        """Continuously read messages and update state until stopped."""
        while not self.stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                time_received = time.time()
                if msg is None:
                    continue
                if msg.get_type().startswith("UNKNOWN"):
                    msg = decode_unknown_message(msg)

                self.state.update(msg)

                # Log all received MAVLink messages with their type and timestamp
                if self.data_logger:
                    self.data_logger.write(
                        {
                            "type": "mavlink_in",
                            "msg_type": msg.get_type(),
                            "data": make_json_safe(msg.to_dict()),
                            "time_received": time_received,
                        }
                    )

            except Exception as exc:
                logging.error("Router failed: %s", exc)

    def send(self, msg: mavlink.MAVLink_message) -> None:
        """Send a MAVLink message and log it."""
        try:
            self.conn.mav.send(msg)

            if self.data_logger:
                self.data_logger.write(
                    {
                        "type": "mavlink_out",
                        "msg_type": msg.get_type(),
                        "data": make_json_safe(msg.to_dict()),
                        "time_sent": time.time(),
                    }
                )

        except Exception as e:
            logging.error(f"MAVLink send error: {e}")
