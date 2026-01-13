"""Guided plan module for UAV missions."""

from __future__ import annotations

from typing import Any, Self

from simulator.helpers.coordinates import ENU, ENUPose, ENUs
from simulator.planner.actions import make_land, make_path, make_takeoff
from simulator.planner.plan import Plan, PlanSpec


@Plan.register("GuidedPlan")
class GuidedPlan(Plan):
    """A UAV guided mode plan to execute waypoints."""

    def __init__(
        self,
        name: str,
        wps: ENUs,
        wp_margin: float = 0.5,
        navigation_speed: float = 5,
        takeoff_alt: float = 1.0,
        land: bool = True,
    ):
        super().__init__(name=name)
        self.extend(Plan.arm(navigation_speed=navigation_speed))
        self.add(make_takeoff(altitude=takeoff_alt))
        self.add(make_path(wps=wps, wp_margin=wp_margin))
        land_wp = ENU(wps[-1].x, wps[-1].y, 0)
        if land:
            self.add(make_land(final_wp=land_wp))

        self._spec = PlanSpec(
            plan_class="GuidedPlan",
            kwargs={
                "name": name,
                "wps": wps,
                "wp_margin": wp_margin,
                "navigation_speed": navigation_speed,
                "takeoff_alt": takeoff_alt,
                "land": land,
            },
        )

    @classmethod
    def from_spec(cls, **kwargs: Any) -> GuidedPlan:
        """Create GuidedPlan from specification dictionary."""
        missing = {"name", "wps"} - kwargs.keys()
        if missing:
            raise ValueError(f"Missing spec fields: {sorted(missing)}")

        enu_wps: ENUs = [ENU(*wp) for wp in kwargs["wps"]]

        return cls(
            name=kwargs["name"],
            wps=enu_wps,
            wp_margin=kwargs.get("wp_margin", 0.5),
            navigation_speed=kwargs.get("navigation_speed", 5),
            takeoff_alt=kwargs.get("takeoff_alt", 1.0),
            land=kwargs.get("land", True),
        )

    @classmethod
    def rectangle_traj(
        cls,
        xlen: float,
        ylen: float,
        alt: float,
        name: str = "guided_rectangle_plan",
        enu_origin: ENUPose = ENUPose(0, 0, 0, 0),
        relative_home: ENUPose = ENUPose(0, 0, 0, 0),
        clockwise: bool = True,
        wp_margin: float = 0.5,
        navigation_speed: float = 5,
        land: bool = True,
    ) -> Self:
        """Create a rectangular guided plan."""
        rel_wps = Plan.create_rectangle_path(
            xlen=xlen, ylen=ylen, alt=alt, clockwise=clockwise
        )
        return cls.from_relative_path(
            relative_path=rel_wps,
            enu_origin=enu_origin,
            relative_home=relative_home,
            name=name,
            wp_margin=wp_margin,
            navigation_speed=navigation_speed,
            land=land,
        )

    @classmethod
    def from_relative_path(
        cls,
        relative_path: ENUs,
        enu_origin: ENUPose = ENUPose(0, 0, 0, 0),
        relative_home: ENUPose = ENUPose(0, 0, 0, 0),
        name: str = "guided_rectangle_plan",
        wp_margin: float = 0.5,
        navigation_speed: float = 5,
        land: bool = True,
    ) -> Self:
        """Create GuidedPlan from relative path."""
        abs_home = enu_origin.to_abs(relative_home)
        abs_path = abs_home.to_abs_all(relative_path)
        wps = ENUPose.unpose_all(abs_path)
        return cls(
            name=name,
            wps=wps,
            wp_margin=wp_margin,
            navigation_speed=navigation_speed,
            land=land,
        )

    @classmethod
    def square_traj(
        cls,
        side_len: float,
        alt: float,
        name: str = "guided_square_plan",
        enu_origin: ENUPose = ENUPose(0, 0, 0, 0),
        relative_home: ENUPose = ENUPose(0, 0, 0, 0),
        clockwise: bool = True,
        wp_margin: float = 0.5,
        navigation_speed: float = 5,
        land: bool = True,
    ) -> Self:
        """Create a square guided plan."""
        return cls.rectangle_traj(
            xlen=side_len,
            ylen=side_len,
            alt=alt,
            enu_origin=enu_origin,
            relative_home=relative_home,
            clockwise=clockwise,
            name=name,
            wp_margin=wp_margin,
            navigation_speed=navigation_speed,
            land=land,
        )
