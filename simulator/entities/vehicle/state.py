"""Thread-safe cache of the latest MAVLink messages and derived vehicle state."""

from __future__ import annotations

import queue
import threading
from collections import defaultdict
from typing import DefaultDict, cast

import pymavlink.dialects.v20.ardupilotmega as mavlink

from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)


class VehicleState:
    """Thread-safe cache of the latest MAVLink messages and derived vehicle state."""

    def __init__(self) -> None:
        self._lock: threading.Lock = threading.Lock()

        # Latest telemetry message per MAVLink type
        self.messages: dict[str, mavlink.MAVLink_message] = {}

        # Transactional message queues (MISSION_REQUEST, COMMAND_ACK, etc.)
        self.queues: DefaultDict[str, queue.Queue[mavlink.MAVLink_message]] = (
            defaultdict(queue.Queue)
        )

    def update(self, msg: mavlink.MAVLink_message) -> None:
        """
        Store a received MAVLink message.

        The message is both:
        - cached as the latest telemetry message of its type
        - pushed into the transactional queue for that message type
        """
        msg_type: str = msg.get_type()
        with self._lock:
            self.messages[msg_type] = msg

        self.queues[msg_type].put(msg)

    def get(self, msg_type: str) -> mavlink.MAVLink_message | None:
        """
        Return the latest cached message of a given MAVLink type.

        This is typically used for telemetry polling.
        """
        with self._lock:
            return self.messages.get(msg_type)

    def wait_for(
        self,
        msg_type: str,
        timeout: float | None = None,
    ) -> mavlink.MAVLink_message | None:
        """Wait for the next message of a given MAVLink type."""
        try:
            return self.queues[msg_type].get(timeout=timeout)
        except queue.Empty:
            return None

    # -------------------------------
    # Factory
    # -------------------------------

    @staticmethod
    def create() -> VehicleStateP:
        """Create a typed VehicleState."""
        return cast(VehicleStateP, VehicleState())
