"""
Defines the Plan class for sequencing UAV actions into structured missions.
Supports static and dynamic waypoint modes and includes predefined plans.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from typing import Any, Callable, ClassVar, TypeVar

from simulator.helpers.connections.mavlink.enums import CopterMode
from simulator.helpers.coordinates import ENU, XY, ENUs, XYs
from simulator.planner.action import Action
from simulator.planner.actions import (
    make_arm,
    make_change_nav_speed,
    make_pre_arm,
    make_set_mode,
)
from simulator.planner.step import Step

P = TypeVar("P", bound="Plan")

ActionSequence = Action[Action[Step]]


@dataclass(frozen=True)
class PlanSpec:
    """Specification for building a Plan."""

    plan_class: str
    kwargs: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        """Convert PlanSpec to a dictionary."""
        return asdict(self)


class Plan(ActionSequence, ABC):
    """A high-level mission plan composed of sequential UAV actions."""

    _REGISTRY: ClassVar[dict[str, type[Plan]]] = {}

    def __init__(
        self,
        name: str,
        emoji: str = "📋",
    ) -> None:
        super().__init__(name, emoji=emoji)
        self._spec: PlanSpec | None = None

    @classmethod
    @abstractmethod
    def from_spec(cls, **kwargs: Any) -> Plan:
        """Build a Plan from JSON-serializable arguments."""
        raise NotImplementedError

    def extend(self, action_suqnece: ActionSequence) -> None:
        """Append another plan's steps to this plan."""
        for action in action_suqnece.steps:
            self.add(action)

    def get_spec(self) -> PlanSpec:
        """Get the specification of this plan."""
        if self._spec is None:
            raise RuntimeError(f"Plan '{self.name}' does not expose a specification")
        return self._spec

    @staticmethod
    def create_rectangle_path(
        xlen: float,
        ylen: float,
        alt: float,
        clockwise: bool = True,
    ) -> ENUs:
        """Create a rectangle path as a list of ENU positions or poses."""
        coords = Plan.create_rectangle_xypath(xlen, ylen, clockwise)
        return [ENU(x, y, alt) for x, y in coords]

    @staticmethod
    def create_square_path(
        side_len: float = 10,
        alt: float = 5,
        clockwise: bool = True,
    ) -> ENUs:
        """Create a square path as a list of ENU positions or poses."""
        return Plan.create_rectangle_path(side_len, side_len, alt, clockwise)

    @staticmethod
    def create_rectangle_xypath(
        xlen: float = 5, ylen: float = 5, clockwise: bool = True
    ) -> XYs:
        """Create square path in XYs."""
        if clockwise:
            coords = XY.list(
                [
                    (0, 0),
                    (0, ylen),
                    (xlen, ylen),
                    (xlen, 0),
                    (0, 0),
                ]
            )
        else:
            coords = XY.list(
                [
                    (0, 0),
                    (xlen, 0),
                    (xlen, ylen),
                    (0, ylen),
                    (0, 0),
                ]
            )
        return coords

    @classmethod
    def arm(
        cls,
        name: str = "ARM",
        navigation_speed: float = 5,
    ) -> ActionSequence:
        """Create a plan to execute a mission in auto mode."""
        actions = ActionSequence(name, emoji="🔐")
        actions.add(make_pre_arm())
        actions.add(make_set_mode(CopterMode.GUIDED))
        if navigation_speed != 5:
            actions.add(make_change_nav_speed(speed=navigation_speed))
        actions.add(make_arm())
        return actions

    @classmethod
    def register(cls, name: str) -> Callable[[type[P]], type[P]]:
        """Register a Plan subclass."""

        def decorator(plan_cls: type[P]) -> type[P]:
            cls._REGISTRY[name] = plan_cls
            return plan_cls

        return decorator

    @classmethod
    def build(cls, spec: PlanSpec) -> Plan:
        """Build a Plan from its specification."""
        plan_cls = cls._REGISTRY[spec.plan_class]
        return plan_cls.from_spec(**spec.kwargs)


Plans = list[Plan]
