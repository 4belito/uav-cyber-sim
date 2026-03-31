"""
Mission execution module defining core classes for steps and actions used
in UAV plans.
"""

from __future__ import annotations

import logging
from enum import StrEnum
from typing import Generic, TypeVar

from simulator.helpers.coordinates import GRA
from simulator.planner.step import MissionElement, State
from simulator.runtime.vehicle.mav_manager import MAVLinkManager

# TODO: Check binding of conn in Action and Step,
# i think conn can be passed when actions/steps are instantiated

# TODO: Treat concurrency Actions better usigng block and timeout

# TODO: Check if NOT_STARTED and DONE may be combined into a single state


T = TypeVar("T", bound=MissionElement)


class Action(MissionElement, Generic[T]):
    """
    Encapsulates a sequence of steps and manages their coordinated
    execution as a mission action.
    """

    class Names(StrEnum):
        """Enumerates standard UAV action types used in mission plans."""

        PREARM = "PREARM"
        ARM = "ARM"
        TAKEOFF = "TAKEOFF"
        FLY = "FLY"
        LAND = "LAND"
        CHANGE_FLIGHTMODE = "MODE"
        CHANGE_NAVSPEED = "CHANGE_NAV_SPEED"
        START_MISSION = "START_MISSION"
        UPLOAD_MISSION = "UPLOAD_MISSION"
        WAIT = "WAIT"
        MONITOR_MISSION = "MONITOR_MISSION"
        AVOIDANCE = "AVOIDANCE"

        @property
        def emoji(self) -> str:
            """Return the emoji representation of the action name."""
            return {
                Action.Names.PREARM: "🛡️",
                Action.Names.ARM: "🔒",
                Action.Names.TAKEOFF: "🛫",
                Action.Names.FLY: "✈️",
                Action.Names.LAND: "🛬",
                Action.Names.CHANGE_FLIGHTMODE: "⚙️",
                Action.Names.CHANGE_NAVSPEED: "🎚️",
                Action.Names.START_MISSION: "🚀",
                Action.Names.UPLOAD_MISSION: "💾",
                Action.Names.WAIT: "🕒",
                Action.Names.MONITOR_MISSION: "👀",
                Action.Names.AVOIDANCE: "🚧",
            }[self]

    def __init__(
        self,
        name: str,
        emoji: str = "📋",
    ) -> None:
        self.steps: list[T] = []
        self.current: T | None = None
        super().__init__(name=name, emoji=emoji)

    def add(self, step: T) -> None:
        """
        Add a Step or Action to this Action/Plan.
        Maintains chaining via `next` and updates current element.
        """
        if self.steps:
            self.steps[-1].next = step
            step.prev = self.steps[-1]
        self.steps.append(step)
        if not self.current:
            self.current = step
        self.target_pos = step.target_pos
        if self.state == State.DONE:
            self.state = State.IN_PROGRESS

    def act(self):
        """Execute current step based on the action's state."""
        if self.state == State.NOT_STARTED:
            self._start_action()
        elif self.state == State.IN_PROGRESS:
            self._progress_action()
        elif self.state == State.DONE:
            self._log_already_done()
        elif self.state == State.FAILED:
            self._log_already_failed()

    def run(self):
        """Run the action until all steps are done or a failure occurs."""
        while self.state not in (State.DONE, State.FAILED):
            self.act()

    def _start_action(self):
        self.state = State.IN_PROGRESS
        logging.debug(
            (
                f"▶️ Vehicle {self.sysid}: {self.class_name} Started: "
                f"{self.emoji} {self.name}"
            )
        )

    def _progress_action(self):
        step = self.current
        if step is None or (step.state == State.DONE and step.next is None):
            self.state = State.DONE
            logging.info(
                (
                    f"✅ Vehicle {self.sysid}: {self.class_name} Done: "
                    f"{self.emoji} {self.name}"
                )
            )
        elif step.state == State.DONE:
            self.current = step.next
        elif step.state == State.FAILED:
            self.state = State.FAILED
            logging.error(
                f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
                f"Already failed! Cannot perform this again!"
            )
        else:
            step.act()
            self.update(step)

    def _log_already_done(self):
        logging.warning(
            f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
            f"Already done! Cannot perform this again!"
        )

    def _log_already_failed(self):
        logging.warning(
            f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
            f"Already failed! Cannot perform this again!"
        )

    def update(self, step: T):
        """Update current position and onair status based on a Step."""
        if step.target_pos is not None:
            self.target_pos = step.target_pos
        if step.curr_pos is not None:
            self.curr_pos = step.curr_pos

    def reset(self) -> None:
        """Reset all steps and set the action state to NOT_STARTED."""
        for step in self.steps:
            step.reset()
        self.current = self.steps[0] if self.steps else None
        super().reset()

    def bind(
        self,
        origin: GRA,
        mav_manager: MAVLinkManager,
    ) -> None:
        """Bind the action to the MAVLink connection."""
        for step in self.steps:
            step.bind(origin, mav_manager)
        super().bind(origin, mav_manager)
        logging.debug(
            f"🔗 Vehicle {self.sysid}: {self.class_name} '{self.name}' is now connected"
        )

    def __repr__(self) -> str:
        output = [super().__repr__()]
        for step in self.steps:
            indented = "\n".join("  " + line for line in repr(step).splitlines())
            output.append(indented)
        return "\n".join(output)
