"""Module defining a Wait step for plans."""

import time

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from plan.core import Step


class Wait(Step):
    """Step to wait for a specified duration."""

    def __init__(self, name: str, t: float) -> None:
        super().__init__(name)
        self.t = t
        self._ready_at: float | None = None

    def exec_fn(self, conn: MAVConnection) -> None:
        """Start the wait timer."""
        self._ready_at = time.monotonic() + self.t

    def check_fn(self, conn: MAVConnection) -> bool:
        """Return True once the wait duration has elapsed."""
        if self._ready_at is None:
            return False
        return time.monotonic() >= self._ready_at
