"""
Mission execution module defining core classes for steps and actions used in UAV plans.
Supports chained execution, state tracking, and verbose status reporting.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Callable, Generic, List, Self, TypeVar, cast

from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU


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


class StepFailed(Exception):
    """Exception raised when a mission step fails due to a known issue."""


# pylint: disable=too-many-instance-attributes
class MissionElement:
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
        self.verbose: int = 1
        self.sysid: int = cast(int, None)

    def act(self):
        """Execute the mission lement action; override in subclasses."""

    def reset(self):
        """Reset the mission element to the NOT_STARTED state."""
        self.state = State.NOT_STARTED

    def __repr__(self) -> str:
        symbol = state_symbols.get(self.state, "❔")
        return f"{symbol} <{self.class_name} '{self.emoji} {self.name}'>"

    def bind(self, connection: MAVConnection, verbose: int = 1) -> None:
        """
        Binds the mission element to a MAVLink connection and sets verbosity
        level.
        """
        self.conn = connection  # Set later from the parent Action
        self.verbose = verbose
        self.sysid = connection.target_system
        if self.verbose > 2:
            print(
                f"Vehicle {self.sysid}: {self.class_name} '{self.name}' is "
                f"now connected ✅🔗"
            )


class Step(MissionElement):
    """Executable mission step with a check and optional execution function."""

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        onair: bool,
        check_fn: Callable[[MAVConnection, int], tuple[bool, ENU | None]],
        exec_fn: Callable[[MAVConnection, int], None],
        target_pos: ENU = ENU(0, 0, 0),
        emoji: str = "🔹",
    ) -> None:
        self.exec_fn = exec_fn
        self.check_fn = check_fn
        self.curr_pos: ENU | None = None
        self.onair = onair
        self.target_pos = target_pos
        super().__init__(name=name, emoji=emoji)

    def execute(self) -> None:
        """Execute the step and change state to IN_PROGRESS."""
        self.exec_fn(self.conn, self.verbose)
        if self.verbose:
            print(f"Vehicle {self.sysid}: ▶️ {self.class_name} Started: {self.name}")
        self.state = State.IN_PROGRESS

    def check(self) -> None:
        """Check step completion and updates state and position."""
        answer, curr_pos = self.check_fn(self.conn, self.verbose)
        if curr_pos is not None:
            self.curr_pos = curr_pos
        if answer:
            self.state = State.DONE
            if self.verbose:
                print(f"Vehicle {self.sysid}: ✅ {self.class_name} Done: {self.name}")

    def act(self):
        """Execute the step or check its progress based on current state."""
        if self.state == State.NOT_STARTED:
            self.execute()
        elif self.state == State.IN_PROGRESS:
            try:
                self.check()
            except StepFailed as e:
                print(
                    f"Vehicle {self.conn.target_system}: ❌ {self.class_name} "
                    f"{self.name} check failed: {e}"
                )
                self.state = State.NOT_STARTED
        elif self.state == State.DONE:
            print("⚠️ Already done!. Cannot perform this step again!")
        elif self.state == State.FAILED:
            print("⚠️ Already failed!. Cannot perform this step again!")

    def reset(self):
        """Reset the step state and clear current position."""
        super().reset()
        self.curr_pos = None

    @staticmethod
    def noop_exec(_conn: MAVConnection, _verbose: int) -> None:
        """No execution."""
        pass

    @staticmethod
    def noop_check(_conn: MAVConnection, _verbose: int) -> tuple[bool, None]:
        """No checking."""
        return True, None


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
        onair: bool | None = None,
        curr_pos: ENU | None = None,
        target_pos: ENU | None = None,
    ) -> None:
        self.steps: List[T] = []
        self.current: T | None = None
        self.onair = onair
        self.curr_pos = curr_pos
        self.target_pos = target_pos
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
        if self.verbose:
            print(
                f"Vehicle {self.sysid}: ▶️ {self.class_name} "
                f"Started: {self.emoji} {self.name}"
            )

    def _progress_action(self):
        step = self.current
        if step is None or (step.state == State.DONE and step.next is None):
            self.state = State.DONE
            if self.verbose:
                print(
                    f"Vehicle {self.sysid}: ✅ {self.class_name} "
                    f"Done: {self.emoji} {self.name}"
                )
        elif step.state == State.DONE:
            self.current = step.next
        elif step.state == State.FAILED:
            self.state = State.FAILED
            print(
                f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
                f"Already failed!. Cannot perform this again!"
            )
        else:
            step.act()
            self.update_pos(step)

    def _log_already_done(self):
        print(
            f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
            f"Already done!. Cannot perform this again!"
        )

    def _log_already_failed(self):
        print(
            f"⚠️ Vehicle {self.sysid}: {self.class_name}: {self.emoji} {self.name} "
            f"Already failed!. Cannot perform this again!"
        )

    def update_pos(self, step: T):
        """Update current position and onair status based on a Step."""
        self.onair = step.onair
        if step.curr_pos is not None:
            self.curr_pos = step.curr_pos

    def reset(self) -> None:
        """Reset all steps and set the action state to NOT_STARTED."""
        for step in self.steps:
            step.reset()
        self.current = self.steps[0] if self.steps else None
        super().reset()

    def bind(self, connection: MAVConnection, verbose: int = 1) -> None:
        """Bind the action to the MAVLink connection and set verbosity."""
        for step in self.steps:
            step.bind(connection, verbose)
        super().bind(connection, verbose)
        if verbose > 2:
            print(
                f"Vehicle {self.sysid}: {self.class_name} '{self.name}' is now "
                f"connected ✅🔗"
            )

    def __repr__(self) -> str:
        output = [super().__repr__()]
        for step in self.steps:
            indented = "\n".join("  " + line for line in repr(step).splitlines())
            output.append(indented)
        return "\n".join(output)
