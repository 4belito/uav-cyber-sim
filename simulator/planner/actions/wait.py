"""Module defining a Wait step for plans."""

import time

from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.planner.action import Action
from simulator.planner.step import Step


class Wait(Step):
    """Step to wait for a specified duration."""

    def __init__(self, name: str, t: float) -> None:
        super().__init__(name)
        self.t = t
        self._ready_at: float | None = None

    def exec_fn(self) -> None:
        """Start the wait timer."""
        self._ready_at = time.monotonic() + self.t

    def check_fn(self) -> bool:
        """Return True once the wait duration has elapsed."""
        if self._ready_at is None:
            return False
        return time.monotonic() >= self._ready_at


class HoldStep(Step):
    """
    Restarts the GLOBAL_POSITION_INT stream (which the preceding GoTo stops)
    and then holds forever so the plan never completes and RID keeps broadcasting.
    """

    def __init__(self, pos_interval: int = 100_000) -> None:
        super().__init__(name="hold position")
        self.pos_interval = pos_interval

    def exec_fn(self) -> None:
        """Re-request position messages that GoTo stopped on arrival."""
        msg = ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.pos_interval)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Never completes — keeps the plan alive so RID keeps broadcasting."""
        return False


def make_hold() -> Action[Step]:
    """Create a WAIT action that restarts position messages and holds indefinitely."""
    name = Action.Names.WAIT
    hold_action: Action[Step] = Action(name=name, emoji=name.emoji)
    hold_action.add(HoldStep())
    return hold_action
