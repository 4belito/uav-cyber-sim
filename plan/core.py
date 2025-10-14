"""
Mission execution module defining core classes for steps and actions used
in UAV plans.
"""

from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from enum import StrEnum
from typing import Generic, List, Self, TypeVar, cast

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.coordinates import ENU

# TODO: Check binding of conn in Action and Step,
# i think conn can be passed when actions/steps are instantiated

# TODO: Treat concurrency Actions better usigng block and timeout

# TODO: Check if NOT_STARTED and DONE may be combined into a single state


class State(StrEnum):
    """Defines possible execution states for mission steps and actions."""

    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"


state_symbols = {
    State.NOT_STARTED: "🕓",
    State.IN_PROGRESS: "🚀",
    State.DONE: "✅",
    State.FAILED: "❌",
}


class ActionNames(StrEnum):
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


class StepFailed(Exception):
    """Exception raised when a mission step fails due to a known issue."""


# pylint: disable=too-many-instance-attributes
class MissionElement(ABC):
    """
    Base class for mission components like steps and actions.
    Handles state, chaining, and verbosity.
    """

    def __init__(self, name: str = "action name", emoji: str = "📝") -> None:
        # General Properties(Step and Action shared)
        self.class_name = self.__class__.__name__
        self.name = name
        self.emoji = emoji
        self.state = State.NOT_STARTED

        ## Building properties
        self.prev: Self | None = None
        self.next: Self | None = None

        ## live property(after building)
        self.conn: MAVConnection = cast(MAVConnection, None)
        self.onair: bool | None = None  # Default onair status
        self.target_pos: ENU | None = None  # Default target position
        self.curr_pos: ENU | None = None  # Default current position
        self.sysid: int = cast(int, None)

    @abstractmethod
    def act(self):
        """Execute the mission lement action; override in subclasses."""
        pass

    def reset(self):
        """Reset the mission element to the NOT_STARTED state."""
        self.state = State.NOT_STARTED

    def __repr__(self) -> str:
        symbol = state_symbols.get(self.state, "❔")
        return f"{symbol} <{self.class_name} '{self.emoji} {self.name}'>"

    def bind(self, connection: MAVConnection) -> None:
        """
        Binds the mission element to a MAVLink connection and sets verbosity
        level.
        """
        self.conn = connection  # Set later from the parent Action
        self.sysid = connection.target_system
        logging.debug(
            f"🔗 Vehicle {self.sysid}: {self.class_name} '{self.name}' is now connected"
        )


class Step(MissionElement, ABC):
    """Executable mission step with a check and optional execution function."""

    def __init__(
        self,
        name: str,
        emoji: str = "🔹",
    ) -> None:
        super().__init__(name=name, emoji=emoji)

    @abstractmethod
    def exec_fn(self, conn: MAVConnection) -> None:
        """Execute the step; override in subclasses."""
        pass

    @abstractmethod
    def check_fn(self, conn: MAVConnection) -> bool:
        """Check if the step is complete; override in subclasses."""
        pass

    def execute(self) -> None:
        """Execute the step and change state to IN_PROGRESS."""
        self.exec_fn(self.conn)
        logging.debug(f"▶️ Vehicle {self.sysid}: {self.class_name} Started: {self.name}")
        self.state = State.IN_PROGRESS

    def check(self) -> None:
        """Check step completion and updates state and position."""
        answer = self.check_fn(self.conn)
        if answer:
            self.state = State.DONE
            logging.debug(
                f"✅ Vehicle {self.sysid}: {self.class_name} Done: {self.name}"
            )

    def act(self):
        """Execute the step or check its progress based on current state."""
        if self.state == State.NOT_STARTED:
            self.execute()
        elif self.state == State.IN_PROGRESS:
            try:
                self.check()
            except StepFailed as e:
                logging.error(
                    f"❌ Vehicle {self.conn.target_system}: {self.class_name} "
                    f"{self.name} check failed: {e}"
                )
                self.state = State.NOT_STARTED
        elif self.state == State.DONE:
            logging.warning("⚠️ Already done! Cannot perform this step again!")
        elif self.state == State.FAILED:
            logging.warning("⚠️ Already failed! Cannot perform this step again!")

    def reset(self):
        """Reset the step state and clear current position."""
        super().reset()
        self.curr_pos = None
        self.onair = None
        self.target_pos = None

    @classmethod
    def make_wait(cls, t: float = 0) -> Step:
        """Wait for t seconds."""

        class Wait(Step):
            def exec_fn(self, conn: MAVConnection) -> None:
                """Wait for t seconds and return True."""
                time.sleep(t)

            def check_fn(self, conn: MAVConnection) -> bool:
                """No checking."""
                return True

        return Wait(name=f"wait {t} seconds", emoji="⏱️")


T = TypeVar("T", bound=MissionElement)


class Action(MissionElement, Generic[T]):
    """
    Encapsulates a sequence of steps and manages their coordinated
    execution as a mission action.
    """

    def __init__(
        self,
        name: str,
        emoji: str = "🔘",
    ) -> None:
        self.steps: List[T] = []
        self.current: T | None = None
        super().__init__(name=name, emoji=emoji)  # ✅ no-op

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
            self.onair = step.onair
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
        if step.onair is not None:
            self.onair = step.onair
        if step.curr_pos is not None:
            self.curr_pos = step.curr_pos

    def reset(self) -> None:
        """Reset all steps and set the action state to NOT_STARTED."""
        for step in self.steps:
            step.reset()
        self.current = self.steps[0] if self.steps else None
        super().reset()

    def bind(self, connection: MAVConnection) -> None:
        """Bind the action to the MAVLink connection."""
        for step in self.steps:
            step.bind(connection)
        super().bind(connection)
        logging.debug(
            f"🔗 Vehicle {self.sysid}: {self.class_name} '{self.name}' is now connected"
        )

    def __repr__(self) -> str:
        output = [super().__repr__()]
        for step in self.steps:
            indented = "\n".join("  " + line for line in repr(step).splitlines())
            output.append(indented)
        return "\n".join(output)
