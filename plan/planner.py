"""
Defines the Plan class for sequencing UAV actions into structured missions.
Supports static and dynamic waypoint modes and includes predefined plans.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Literal, overload

from mavlink.customtypes.location import (
    ENU,
    XY,
    ENUPose,
    ENUPoses,
    ENUs,
    XYs,
)
from mavlink.enums import CopterMode
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


class Plan(Action[Action[Step]]):
    """A high-level mission plan composed of sequential UAV actions."""

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def __init__(
        self,
        name: str,
        emoji: str = "📋",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: ENUs | None = None,
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
        super().__init__(name, emoji=emoji, curr_pos=ENU(0, 0, 0))

    @staticmethod
    @overload
    def create_square_path(
        side_len: float,
        alt: float,
        clockwise: bool,
        out_offset: Literal[False],
    ) -> ENUs: ...

    @staticmethod
    @overload
    def create_square_path(
        side_len: float,
        alt: float,
        clockwise: bool,
    ) -> ENUs: ...

    @staticmethod
    @overload
    def create_square_path(
        side_len: float,
        alt: float,
    ) -> ENUs: ...

    @staticmethod
    @overload
    def create_square_path(
        side_len: float,
        alt: float,
        clockwise: bool,
        out_offset: Literal[True],
    ) -> ENUPoses: ...

    @staticmethod
    def create_square_path(
        side_len: float = 10,
        alt: float = 5,
        clockwise: bool = True,
        out_offset: bool = False,
    ) -> ENUs | ENUPoses:
        """Create a square path as a list of ENU positions or poses."""
        coords = Plan.create_square_path_mapcoords(side_len, clockwise)
        if out_offset:
            return [ENUPose(x, y, alt, 0.0) for x, y in coords]
        return [ENU(x, y, alt) for x, y in coords]

    @staticmethod
    def create_square_path_mapcoords(
        side_len: float = 10, clockwise: bool = True
    ) -> XYs:
        """Create square path in XYs."""
        if clockwise:
            coords = [
                XY(0, 0),
                XY(0, side_len),
                XY(side_len, side_len),
                XY(side_len, 0),
                XY(0, 0),
            ]
        else:
            coords = [
                XY(0, 0),
                XY(side_len, 0),
                XY(side_len, side_len),
                XY(0, side_len),
                XY(0, 0),
            ]
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
        wps = cls.create_square_path(side_len, alt, clockwise)
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
        wps: ENUs = [ENU(0.0, 0.0, 5.0)],
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "basic",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: ENUs | None = None,
        takeoff_alt: float = 1.0,
    ) -> Plan:
        """Create a basic plan with configurable waypoints and options."""
        land_wp = ENU(wps[-1][0], wps[-1][1], 0)
        plan = cls(name=name, mode=mode, dynamic_wps=dynamic_wps, wp_margin=wp_margin)
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))

        plan.add(make_land(final_wp=land_wp))
        return plan

    # TODO Improve this for no repeating code

    @classmethod
    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-positional-arguments
    def hover(
        cls,
        wps: ENUs | None = None,
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

    # TODO include here the mission somehow. Maybe by passing a name file argument

    @classmethod
    def auto(
        cls,
        name: str = "",
        mission_name: str = "misison",
        from_scratch: bool = True,
        check_until: int = 7,
    ):
        """Create a plan to execute a mission in auto mode."""
        plan = cls(name)
        plan.add(make_upload_mission(mission_name, from_scratch))
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        plan.add(make_arm())
        plan.add(make_start_mission())
        if check_until:
            plan.add(make_monitoring(check_until))
        return plan


Plans = list[Plan]
