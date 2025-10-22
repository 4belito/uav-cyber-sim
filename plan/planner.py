"""
Defines the Plan class for sequencing UAV actions into structured missions.
Supports static and dynamic waypoint modes and includes predefined plans.
"""

from __future__ import annotations

from enum import StrEnum
from typing import overload

from helpers.connections.mavlink.enums import CopterMode
from helpers.coordinates import ENU, XY, ENUPose, ENUPoses, ENUs, XYs
from plan.actions import (
    make_arm,
    make_change_nav_speed,
    make_land,
    make_monitoring,
    make_path,
    make_pre_arm,
    make_set_mode,
    make_start_mission,
    make_takeoff,
    make_upload_mission,
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


# TODO: move annotations out


class Plan(Action[Action[Step]]):
    """A high-level mission plan composed of sequential UAV actions."""

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        emoji: str = "📋",
    ) -> None:
        super().__init__(name, emoji=emoji)

    @staticmethod
    @overload
    def create_rectangle_path(
        xlen: float,
        ylen: float,
        alt: float,
        heading: int,
        clockwise: bool = ...,
    ) -> ENUPoses: ...

    @staticmethod
    @overload
    def create_rectangle_path(
        xlen: float,
        ylen: float,
        alt: float,
        heading: None = ...,
        clockwise: bool = ...,
    ) -> ENUs: ...

    @staticmethod
    def create_rectangle_path(
        xlen: float,
        ylen: float,
        alt: float,
        heading: None | int = None,
        clockwise: bool = True,
    ) -> ENUs | ENUPoses:
        """Create a rectangle path as a list of ENU positions or poses."""
        coords = Plan.create_rectangle_xypath(xlen, ylen, clockwise)
        if heading is not None:
            return [ENUPose(x, y, alt, heading) for x, y in coords]
        return [ENU(x, y, alt) for x, y in coords]

    @staticmethod
    @overload
    def create_square_path(
        heading: int,
        side_len: float = ...,
        alt: float = ...,
        clockwise: bool = ...,
    ) -> ENUPoses: ...

    @staticmethod
    @overload
    def create_square_path(
        heading: None = ...,
        side_len: float = ...,
        alt: float = ...,
        clockwise: bool = ...,
    ) -> ENUs: ...

    @staticmethod
    def create_square_path(
        heading: int | None = None,
        side_len: float = 10,
        alt: float = 5,
        clockwise: bool = True,
    ) -> ENUs | ENUPoses:
        """Create a square path as a list of ENU positions or poses."""
        return Plan.create_rectangle_path(side_len, side_len, alt, heading, clockwise)

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
    def square(
        cls,
        side_len: float = 10,
        alt: float = 5,
        wp_margin: float = 0.5,
        clockwise: bool = True,
        navegation_speed: float = 5,
    ):
        """Create a square-shaped trajectory with takeoff and landing."""
        wps = cls.create_square_path(side_len=side_len, alt=alt, clockwise=clockwise)
        return cls.basic(
            wps=wps,
            wp_margin=wp_margin,
            navegation_speed=navegation_speed,
            name="Square Trajectory",
        )

    @classmethod
    def basic(
        cls,
        wps: ENUs = [ENU(0.0, 0.0, 5.0)],
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "basic",
        takeoff_alt: float = 1.0,
    ) -> Plan:
        """Create a basic plan with configurable waypoints and options."""
        land_wp = ENU(wps[-1][0], wps[-1][1], 0)
        plan = cls(name=name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))

        plan.add(make_land(final_wp=land_wp))
        return plan

    # TODO Add concatenate plan/actiom method to no repeat code

    @classmethod
    def hover(
        cls,
        wps: ENUs,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "hover",
        takeoff_alt: float = 5.0,
    ):
        """Create a plan to take off, reach a point, and hover."""
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        return plan

    @classmethod
    def auto(
        cls,
        name: str,
        mission_path: str,
        mission_delay: float = 0,
        from_scratch: bool = True,
        navegation_speed: float = 5,
    ):
        """Create a plan to execute a mission in auto mode."""
        plan = cls(name)
        plan.add(make_upload_mission(mission_path, from_scratch))
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_start_mission())
        plan.add(make_monitoring())
        return plan

    @classmethod
    def arm(
        cls,
        name: str,
        mission_delay: float = 0,
    ):
        """Create a plan to execute a mission in auto mode."""
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        plan.add(make_arm())
        return plan


Plans = list[Plan]
