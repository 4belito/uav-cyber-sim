"""
Mission execution module defining core classes for steps and actions used in UAV plans.
Supports chained execution, state tracking, and verbose status reporting.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Callable, List, Optional, Self, cast

from helpers.change_coordinates import Position
from plan.mav_helpres import MAVConnection


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


class StepFailed(Exception):
    """Exception raised when a mission step fails due to a known issue."""


# pylint: disable=too-many-instance-attributes
class MissionElement:
    """
    Base class for mission components like steps and actions.
    Handles state, chaining, and verbosity.
    """

    def __init__(
        self, name: str = "action name", emoji: str = "📝", is_improv: bool = False
    ) -> None:
        # General Properties(Step and Action shared)
        self.class_name = self.__class__.__name__
        self.name = name
        self.emoji = emoji
        self.is_improv = is_improv
        self.state = State.NOT_STARTED

        ## Building properties
        self.prev: Self | None = None
        self.next: Self | None = None

        ## live property(after building)
        self.conn: MAVConnection = cast(MAVConnection, None)
        self.verbose: int = 1
        self.sysid: int = cast(int, None)

    def act(self):
        """Base method for mission logic; override in subclasses."""

    def reset(self):
        """Resets element to NOT_STARTED state."""
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
        if self.verbose > 2:
            print(
                f"Vehicle {self.sysid}: {self.class_name} '{self.name}' is "
                f"now connected ✅🔗"
            )


def _noop_exec(_: MAVConnection) -> None:
    pass


class Step(MissionElement):
    """Executable mission step with a check and optional execution function."""

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        onair: bool,
        check_fn: Callable[[MAVConnection, int], tuple[bool, Optional[Position]]],
        exec_fn: Optional[Callable[[MAVConnection], None]] = None,
        target_pos: Position = (0, 0, 0),
        emoji: str = "🔹",
        is_improv: bool = False,
    ) -> None:
        self.exec_fn: Callable[[MAVConnection], None] = exec_fn or _noop_exec
        self.check_fn: Callable[
            [MAVConnection, int], tuple[bool, Optional[Position]]
        ] = check_fn
        self.curr_pos: Optional[Position] = None
        self.onair = onair
        self.target_pos = target_pos
        super().__init__(name=name, emoji=emoji, is_improv=is_improv)

    def execute(self) -> None:
        """Executes the step and marks it as IN_PROGRESS."""
        self.exec_fn(self.conn)
        if self.verbose:
            print(f"Vehicle {self.sysid}: ▶️ {self.class_name} Started: {self.name}")
        self.state = State.IN_PROGRESS

    def check(self) -> None:
        """Checks step completion and updates state and position."""
        answer, curr_pos = self.check_fn(self.conn, self.verbose)
        if curr_pos is not None:
            self.curr_pos = curr_pos
        if answer:
            self.state = State.DONE
            if self.verbose:
                print(f"Vehicle {self.sysid}: ✅ {self.class_name} Done: {self.name}")

    def act(self):
        if self.state == State.NOT_STARTED:
            self.execute()
        elif self.state == State.IN_PROGRESS:
            self.check()
        elif self.state == State.DONE:
            print("⚠️ Already done!. Cannot perform this step again!")
        elif self.state == State.FAILED:
            print("⚠️ Already failed!. Cannot perform this step again!")

    def reset(self):
        super().reset()
        self.curr_pos = None


class Action(MissionElement):
    """
    Encapsulates a sequence of steps and manages their coordinated
    execution as a mission action.
    """

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        emoji: str = "🔘",
        onair: bool | None = None,
        curr_pos: Position | None = None,
        target_pos: Position | None = None,
        is_improv: bool = False,
    ) -> None:
        self.steps: List[Step] = []
        self.current: Step | None = None
        self.onair = onair
        self.curr_pos = curr_pos
        self.target_pos = target_pos
        super().__init__(name=name, emoji=emoji, is_improv=is_improv)  # ✅ no-op

    def add_step(self, step: Step) -> None:
        """
        Adds a Step or Action to this Action/Plan.
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

    def update_pos(self, step: Step):
        """Updates current position and onair status based on a Step."""
        self.onair = step.onair
        if step.curr_pos is not None:
            self.curr_pos = step.curr_pos

    def reset(self) -> None:
        # Change the stated of the steps to no started
        for step in self.steps:
            step.reset()
        # change current to the first step
        self.current = self.steps[0] if self.steps else None
        # change the action state to no statrted
        super().reset()

    def bind(self, connection: MAVConnection, verbose: int = 1) -> None:
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

    def add_next(self, new_step: Step) -> None:
        """
        Inserts a new step/action immediately after the current step.
        Maintains chaining and updates the 'next' pointers accordingly.
        current<new<next
        """
        if self.current is None:
            # If no current step exists, treat it like a normal add
            self.add_step(new_step)
            return

        next_step = self.current.next  # save next step
        self.current.next = new_step  # current -> new
        new_step.prev = self.current  # current <- new
        new_step.next = next_step  #     new -> next
        if next_step:
            next_step.prev = new_step  #    new <- next

        # Insert in the list just after the current step
        current_index = self.steps.index(self.current)
        self.steps.insert(current_index + 1, new_step)

    def add_prev(self, new_step: Step) -> None:
        """
        Inserts a new step/action immediately before the current step.
        Maintains chaining and updates the 'next' pointers accordingly.
        prev<new<current
        """
        if self.current is None:
            # If no current step exists, treat it like a normal add
            self.add_step(new_step)
            return

        prev_step = self.current.prev  # save prev_step
        new_step.next = self.current  # new -> current
        new_step.prev = prev_step  # prev <- new
        self.current.prev = new_step  # new <- current
        if prev_step:
            prev_step.next = new_step  # prev -> new

        current_index = self.steps.index(self.current)
        self.steps.insert(current_index, new_step)

    def add_over(self, new_step: Step) -> None:
        """Insert a step after current and mark current step as done."""
        self.add_next(new_step)
        self.current.state = State.DONE  # type: ignore[union-attr]
        self.current = new_step

    def add_now(self, new_step: Step) -> None:
        """Insert a step before current and reset it for immediate execution."""
        self.add_prev(new_step)
        self.current.reset()  # type: ignore[union-attr]
        self.current = new_step
