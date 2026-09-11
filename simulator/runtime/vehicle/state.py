"""Thread-safe cache of the latest MAVLink messages and derived vehicle state."""

from __future__ import annotations

import queue
import threading
from collections import defaultdict
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    import pymavlink.dialects.v20.ardupilotmega as mavlink

    from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
        VehicleStateP,
    )

# Message types carrying `time_boot_ms` — the autopilot's boot clock, which runs
# at SITL `speedup`. Ordered most-frequent-first.
_SIM_CLOCK_TYPES = ("GLOBAL_POSITION_INT", "ATTITUDE", "SYSTEM_TIME")


class VehicleState:
    """Thread-safe cache of the latest MAVLink messages and derived vehicle state."""

    def __init__(self) -> None:
        self._lock: threading.Lock = threading.Lock()

        # Latest telemetry message per MAVLink type
        self.messages: dict[str, mavlink.MAVLink_message] = {}
        # TODO: Check if queues is being used
        # Transactional message queues (MISSION_REQUEST, COMMAND_ACK, etc.)
        self.queues: defaultdict[str, queue.Queue[mavlink.MAVLink_message]] = (
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

    def sim_time_s(self) -> float | None:
        """
        Return the vehicle's boot clock in seconds, or `None` if no timestamped
        message has arrived yet.

        It advances at SITL `speedup` (and slows with the sim if the host can't
        keep up), so timing measured against it tracks mission progress rather
        than the wall clock.
        """
        for msg_type in _SIM_CLOCK_TYPES:
            boot_ms = getattr(self.get(msg_type), "time_boot_ms", None)
            if boot_ms is not None:
                return float(boot_ms) / 1000.0
        return None

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

    @staticmethod
    def create() -> VehicleStateP:
        """Create a typed VehicleState."""
        return cast("VehicleStateP", VehicleState())
