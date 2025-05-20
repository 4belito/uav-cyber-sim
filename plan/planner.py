"""
Defines the Plan class for sequencing UAV actions into structured missions.
Supports static and dynamic waypoint modes and includes predefined plans.
"""

from __future__ import annotations

from enum import StrEnum
import numpy as np
from numpy.typing import NDArray

from plan.actions import (
    make_arm,
    make_change_nav_speed,
    make_land,
    make_path,
    make_pre_arm,
    make_set_mode,
    make_takeoff,
)
from plan.core import Step

from .core import Action


class State(StrEnum):
    """Represents the execution status of a mission element."""

    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"


class PlanMode(StrEnum):
    """Defines whether a plan uses static or dynamic waypoints."""

    DYNAMIC = "DYNAMIC"
    STATIC = "STATIC"


class Plan(Action[Action[Step]]):
    """A high-level mission plan composed of sequential UAV actions."""

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        emoji: str = "📋",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: NDArray[np.float64] | None = None,
        wp_margin: float = 0.5,
    ) -> None:
        self.mode = mode
        self.wp_margin = wp_margin
        if mode == PlanMode.STATIC:
            if dynamic_wps is not None:
                raise ValueError(
                    "Do not provide dynamic_waypoints (dynamic_wps) for STATIC plans."
                )
            self.dynamic_wps = None
        elif mode == PlanMode.DYNAMIC:
            if dynamic_wps is None:
                raise ValueError(
                    "Dynamic waypoints (dynamic_wps) are required for DYNAMIC plans."
                )
            self.dynamic_wps = dynamic_wps
        else:
            raise ValueError(f"Unsupported plan mode: {mode}")
        super().__init__(name, emoji=emoji, curr_pos=(0, 0, 0))

    @staticmethod
    def create_square_path(
        side_len: float = 10, alt: float = 5, clockwise: bool = True
    ):
        """Generates square-shaped waypoints for a trajectory."""
        if clockwise:
            wps = np.array(
                [
                    (0, 0, alt),
                    (0, side_len, alt),
                    (side_len, side_len, alt),
                    (side_len, 0, alt),
                    (0, 0, alt),
                ]
            )
        else:
            wps = np.array(
                [
                    (0, 0, alt),
                    (0, side_len, alt),
                    (-side_len, side_len, alt),
                    (-side_len, 0, alt),
                    (0, 0, alt),
                ]
            )

        return wps

    @classmethod
    def square(
        cls,
        side_len: float = 10,
        alt: float = 5,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
    ):
        """Creates a square-shaped trajectory with takeoff and landing."""
        wps = cls.create_square_path(side_len, alt)
        return cls.basic(
            wps=wps,
            wp_margin=wp_margin,
            navegation_speed=navegation_speed,
            name="Square Trajectory",
        )

    @classmethod
    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def basic(
        cls,
        wps: NDArray[np.float64] = np.array([[0, 0, 5]]),
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "basic",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: NDArray[np.float64] | None = None,
        takeoff_alt: float = 1.0,
    ) -> Plan:
        """Creates a basic plan with configurable waypoints and options."""
        land_wp = wps[-1].copy()
        land_wp[2] = 0
        plan = cls(name=name, mode=mode, dynamic_wps=dynamic_wps, wp_margin=wp_margin)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))

        plan.add(make_land(final_wp=land_wp))
        return plan

    ## Improve this for no repeating code

    @classmethod
    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def hover(
        cls,
        wps: NDArray[np.float64] | None = None,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "hover",
        takeoff_alt: float = 5.0,
    ):
        """Creates a plan to take off, reach a point, and hover."""
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        return plan
