"""Module defining a Wait step for plans."""

from __future__ import annotations

from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.planner.action import Action
from simulator.planner.step import Step


class Wait(Step):
    """
    Step that waits `t` **sim** seconds (vehicle boot clock), so the wait is
    unaffected by `speedup` or host load.
    """

    def __init__(self, name: str, t: float) -> None:
        super().__init__(name)
        self.t = t
        self._deadline: float | None = None

    def exec_fn(self) -> None:
        """Ensure a timestamped stream is flowing; the deadline is set on first tick."""
        msg = ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT, interval=100_000)
        self.mav_manager.send(msg)
        self._deadline = None

    def check_fn(self) -> bool:
        """Return True once `t` sim seconds have elapsed."""
        now = self.mav_manager.state.sim_time_s()
        if now is None:
            return False
        if self._deadline is None:
            self._deadline = now + self.t
        return now >= self._deadline


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
