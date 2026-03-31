"""
Mission execution module defining core classes for steps and actions used
in UAV plans.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from enum import StrEnum
from typing import Self

from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)
from simulator.helpers.coordinates import ENU, GRA
from simulator.runtime.vehicle.mav_manager import MAVLinkManager


class State(StrEnum):
    """Defines possible execution states for mission steps and actions."""

    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"

    @property
    def emoji(self) -> str:
        """Return the emoji representation of the state."""
        return {
            State.NOT_STARTED: "🕓",
            State.IN_PROGRESS: "🚀",
            State.DONE: "✅",
            State.FAILED: "❌",
        }[self]


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
        self.origin: GRA
        self.mav_manager: MAVLinkManager
        self.onair: bool | None = None  # Default onair status
        self.target_pos: ENU | None = None  # Default target (global) position
        self.curr_pos: ENU | None = None  # Default current (global) position

    @abstractmethod
    def act(self):
        """Execute the mission lement action; override in subclasses."""
        pass

    def reset(self):
        """Reset the mission element to the NOT_STARTED state."""
        self.state = State.NOT_STARTED

    def __repr__(self) -> str:
        return f"{self.state.emoji} <{self.class_name} '{self.emoji} {self.name}'>"

    def bind(
        self,
        origin: GRA,
        mav_manager: MAVLinkManager,
    ) -> None:
        """
        Binds the mission element to a MAVLink connection, origin, and vehicle
        state.
        """
        self.origin = origin
        self.mav_manager = mav_manager
        logging.debug(
            f"🔗 Vehicle {self.sysid}: {self.class_name} '{self.name}' is now connected"
        )

    @property
    def conn(self) -> MAVConnection:
        """Convenience property to access the MAVLink connection."""
        assert self.mav_manager is not None, (
            "Mission must be bound to access connection"
        )
        return self.mav_manager.conn

    @property
    def vehicle_state(self) -> VehicleStateP:
        """Convenience property to access the shared vehicle state."""
        assert self.mav_manager is not None, (
            "Mission must be bound to access vehicle state"
        )
        return self.mav_manager.state

    @property
    def sysid(self) -> int:
        """Convenience property to access the vehicle sysid."""
        assert self.mav_manager is not None, "Mission must be bound to access sysid"
        return self.mav_manager.data_logger.sysid


class Step(MissionElement, ABC):
    """Executable mission step with a check and optional execution function."""

    def __init__(
        self,
        name: str,
    ) -> None:
        super().__init__(name=name, emoji="🔹")

    @abstractmethod
    def exec_fn(self) -> None:
        """Execute the step; override in subclasses."""
        pass

    @abstractmethod
    def check_fn(self) -> bool:
        """Check if the step is completed; override in subclasses."""
        pass

    def execute(self) -> None:
        """Execute the step and change state to IN_PROGRESS."""
        self.exec_fn()
        logging.debug(f"▶️ Vehicle {self.sysid}: {self.class_name} Started: {self.name}")
        self.state = State.IN_PROGRESS

    def check(self) -> None:
        """Check step completion and updates state and position."""
        answer = self.check_fn()
        if answer:
            self.state = State.DONE
            logging.debug(
                f"✅ Vehicle {self.sysid}: {self.class_name} Done: {self.name}"
            )

    def run(self):
        """Execute the step or check its progress based on current state."""
        while self.state not in [State.DONE, State.FAILED]:
            self.act()

    def act(self):
        """Execute the step or check its progress based on current state."""
        if self.state == State.NOT_STARTED:
            self.execute()
        elif self.state == State.IN_PROGRESS:
            try:
                self.check()
            except Exception as exc:
                logging.error(
                    "❌ Vehicle %s: %s %s check failed: %s",
                    self.conn.target_system,
                    self.class_name,
                    self.name,
                    exc,
                )
                self.state = State.FAILED
        elif self.state == State.DONE:
            logging.warning("⚠️ Already done! Cannot perform this step again!")
        elif self.state == State.FAILED:
            logging.warning("⚠️ Already failed! Cannot perform this step again!")

    def get_enu_position(self) -> ENU | None:
        """Get the current ENU position of the UAV."""
        msg = self.vehicle_state.get("GLOBAL_POSITION_INT")
        if msg:
            gra_pos = GRA.from_global_int(msg.lat, msg.lon, msg.alt)
            pos = self.origin.to_rel(gra_pos)
            return pos
        return None

    def reset(self):
        """Reset the step state and clear current position."""
        super().reset()
        self.curr_pos = None
        self.onair = None
        self.target_pos = None
