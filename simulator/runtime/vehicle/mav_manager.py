"""
Single MAVLink reader.
Reads from the connection and updates the shared vehicle state.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Sequence

import pymavlink.dialects.v20.ardupilotmega as mavlink

from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.streams import (
    decode_unknown_message,
    make_json_safe,
)
from simulator.helpers.logging.data_logger import DataLogger
from simulator.runtime.vehicle.state import VehicleState

# SITL messages forwarded to the GCS telemetry channel.
_GCS_TELEMETRY_TYPES: frozenset[str] = frozenset(
    {
        "HEARTBEAT",
        "GLOBAL_POSITION_INT",
        "MISSION_CURRENT",
        "STATUSTEXT",
        "VFR_HUD",
        "ATTITUDE",
        "SYS_STATUS",
    }
)


class MAVLinkManager(threading.Thread):
    """
    MAVLink I/O manager.

    - Receives MAVLink messages (RX)
    - Sends MAVLink messages (TX)
    - Logs all traffic
    - Forwards telemetry to every GCS connection it was given (possibly none)
    """

    def __init__(
        self,
        conn: MAVConnection,
        data_logger: DataLogger,
        gcs_conns: Sequence[MAVConnection] = (),
    ) -> None:
        super().__init__(daemon=True)
        self.conn = conn
        self.state = VehicleState.create()
        self._stop_event = threading.Event()
        self.data_logger = data_logger
        self.gcs_conns = list(gcs_conns)

    def run(self) -> None:
        """Continuously read messages and update state until stopped."""
        while not self._stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.1)
                time_received = time.time()
                if msg is None:
                    continue
                if msg.get_type().startswith("UNKNOWN"):
                    msg = decode_unknown_message(msg)

                # Heartbeats from non-autopilot sources (e.g. echoed GCS/logic
                # heartbeats routed back by ArduPilot) must not overwrite the
                # vehicle autopilot's HEARTBEAT in state.  The autopilot always
                # uses srcComponent == 1 (MAV_COMP_ID_AUTOPILOT1).
                if msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() != 1:
                    continue

                self.state.update(msg)

                if msg.get_type() in _GCS_TELEMETRY_TYPES:
                    for gcs_conn in self.gcs_conns:
                        try:
                            gcs_conn.mav.send(msg)
                        except Exception as fwd_exc:
                            logging.debug("GCS telemetry forward error: %s", fwd_exc)

                # Log all received MAVLink messages with their type and timestamp
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

    def stop(self) -> None:
        """Signal the thread to stop."""
        self._stop_event.set()
        self.join()
        self.conn.close()

    def send(self, msg: mavlink.MAVLink_message) -> None:
        """Send a MAVLink message and log it."""
        try:
            self.conn.mav.send(msg)

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
