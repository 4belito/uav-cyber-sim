"""
Single MAVLink reader.
Reads from the connection and updates the shared vehicle state.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from simulator.entities.riddata import RIDDict
from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)
from simulator.helpers.connections.mavlink.streams import decode_unknown_message

DataWriter = Callable[
    [dict[str, int | float | str | RIDDict | dict[str, str | int | float]]], None
]


class MAVLinkRouter(threading.Thread):
    """
    Single MAVLink reader.
    Reads from the connection and updates the shared vehicle state.
    """

    def __init__(
        self,
        conn: MAVConnection,
        state: VehicleStateP,
        stop_event: threading.Event,
        data_writer: DataWriter,
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.state = state
        self.stop_event = stop_event
        self.data_writer = data_writer

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
                try:
                    self.data_writer(
                        {
                            "type": "ardupilot_msg",
                            "msg_type": msg.get_type(),
                            "data": msg.to_dict(),
                            "time_received": time_received,
                        }
                    )
                except Exception as e:
                    logging.error(f"MAVLink data write error: {e}")

            except Exception as exc:
                logging.error("Router failed: %s", exc)
