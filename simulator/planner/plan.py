"""
Defines the Plan class for sequencing UAV actions into structured missions.
Supports static and dynamic waypoint modes and includes predefined plans.
"""

from __future__ import annotations

from simulator.helpers.connections.mavlink.enums import CopterMode
from simulator.helpers.coordinates import ENU, XY, ENUs, XYs
from simulator.planner.action import Action
from simulator.planner.actions import (
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
from simulator.planner.step import Step


class Plan(Action[Action[Step]]):
    """A high-level mission plan composed of sequential UAV actions."""

    def __init__(
        self,
        name: str,
        emoji: str = "📋",
    ) -> None:
        super().__init__(name, emoji=emoji)

    def extend(self, plan: Plan) -> None:
        """Append another plan's steps to this plan."""
        for action in plan.steps:
            self.add(action)

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
    def arm(
        cls,
        name: str = "ARM",
        navegation_speed: float = 5,
    ):
        """Create a plan to execute a mission in auto mode."""
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode(CopterMode.GUIDED))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        return plan

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
        plan = cls.arm(name=name, navegation_speed=navegation_speed)
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        return plan

    @classmethod
    def basic(
        cls,
        wps: ENUs,
        name: str = "basic",
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        takeoff_alt: float = 1.0,
    ) -> Plan:
        """Create a basic plan with configurable waypoints and options."""
        land_wp = ENU(wps[-1][0], wps[-1][1], 0)
        plan = cls.hover(
            name=name,
            navegation_speed=navegation_speed,
            wps=wps,
            wp_margin=wp_margin,
            takeoff_alt=takeoff_alt,
        )
        plan.add(make_land(final_wp=land_wp))
        return plan

    @classmethod
    def auto(
        cls,
        name: str,
        mission_path: str,
        from_scratch: bool = True,
        navegation_speed: float = 5,
    ):
        """Create a plan to execute a mission in auto mode."""
        plan = cls(name)
        plan.add(make_upload_mission(mission_path, from_scratch))
        plan.extend(
            cls.arm(
                name="auto_arm",
                navegation_speed=navegation_speed,
            )
        )
        plan.add(make_start_mission())
        plan.add(make_monitoring())
        return plan


Plans = list[Plan]
