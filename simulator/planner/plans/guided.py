"""Guided plan module for UAV missions."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Self

from simulator.helpers.coordinates import ENU, ENUPose, ENUs
from simulator.helpers.math import enu_bearing
from simulator.planner.actions import (
    make_hold,
    make_land,
    make_path,
    make_takeoff,
)
from simulator.planner.plan import Plan, PlanSpec

if TYPE_CHECKING:
    from simulator.config import Firmware


@Plan.register("GuidedPlan")
class GuidedPlan(Plan):
    """A UAV guided mode plan to execute waypoints."""

    def __init__(
        self,
        name: str,
        wps: ENUs,
        firmware: Firmware,
        wp_margin: float | None = None,
        navigation_speed: float = 5,
        takeoff_alt: float = 1.0,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
        land: bool = True,
    ):
        super().__init__(name=name)
        # Skip home (first) and last WP for navigation — matches AutoPlan's
        # save_basic_mission which uses wps[1:-1] for nav waypoints.
        nav_wps = wps[1:-1] if len(wps) > 2 else wps
        # Raw world-bearing; home_heading subtraction deferred to PlaneTakeOff.exec_fn.
        if land_bearing is None and firmware == "ArduPlane" and len(wps) >= 2:
            land_bearing = enu_bearing(wps[-2], wps[-1])
        self.extend(Plan.arm(navigation_speed=navigation_speed, firmware=firmware))
        self.add(
            make_takeoff(
                altitude=takeoff_alt,
                firmware=firmware,
                land_bearing=land_bearing,
            )
        )
        self.add(make_path(wps=nav_wps, wp_margin=wp_margin, firmware=firmware))
        if not land:
            self.add(make_hold())
        elif firmware == "ArduPlane":
            land_wp = ENUPose(wps[-1].x, wps[-1].y, 0, land_bearing or 0.0)
            self.add(
                make_land(
                    land_wp=land_wp,
                    firmware=firmware,
                    autoland_alt=autoland_alt,
                    autoland_wp_dist=autoland_wp_dist,
                )
            )
        else:
            self.add(make_land(firmware=firmware))

        self._spec = PlanSpec(
            plan_class="GuidedPlan",
            kwargs={
                "name": name,
                "wps": wps,
                "wp_margin": wp_margin,
                "navigation_speed": navigation_speed,
                "takeoff_alt": takeoff_alt,
                "land_bearing": land_bearing,
                "autoland_alt": autoland_alt,
                "autoland_wp_dist": autoland_wp_dist,
                "land": land,
                "firmware": firmware,
            },
        )

    @classmethod
    def from_spec(cls, **kwargs: Any) -> GuidedPlan:
        """Create GuidedPlan from specification dictionary."""
        missing = {"name", "wps", "firmware"} - kwargs.keys()
        if missing:
            raise ValueError(f"Missing spec fields: {sorted(missing)}")

        enu_wps: ENUs = [ENU(*wp) for wp in kwargs["wps"]]

        return cls(
            name=kwargs["name"],
            wps=enu_wps,
            wp_margin=kwargs.get("wp_margin"),
            navigation_speed=kwargs.get("navigation_speed", 5),
            takeoff_alt=kwargs.get("takeoff_alt", 1.0),
            land_bearing=kwargs.get("land_bearing"),
            autoland_alt=kwargs.get("autoland_alt"),
            autoland_wp_dist=kwargs.get("autoland_wp_dist"),
            land=kwargs.get("land", True),
            firmware=kwargs["firmware"],
        )

    @classmethod
    def rectangle_traj(
        cls,
        xlen: float,
        ylen: float,
        alt: float,
        firmware: Firmware,
        name: str = "guided_rectangle_plan",
        enu_origin: ENUPose | None = None,
        relative_home: ENUPose | None = None,
        clockwise: bool = True,
        wp_margin: float | None = None,
        navigation_speed: float = 5,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
        land: bool = True,
    ) -> Self:
        """Create a rectangular guided plan."""
        if enu_origin is None:
            enu_origin = ENUPose(0, 0, 0, 0)
        if relative_home is None:
            relative_home = ENUPose(0, 0, 0, 0)
        rel_wps = Plan.create_rectangle_path(
            xlen=xlen, ylen=ylen, alt=alt, clockwise=clockwise
        )
        return cls.from_relative_path(
            relative_path=rel_wps,
            firmware=firmware,
            enu_origin=enu_origin,
            relative_home=relative_home,
            name=name,
            wp_margin=wp_margin,
            navigation_speed=navigation_speed,
            land_bearing=land_bearing,
            autoland_alt=autoland_alt,
            autoland_wp_dist=autoland_wp_dist,
            land=land,
        )

    @classmethod
    def from_relative_path(
        cls,
        relative_path: ENUs,
        firmware: Firmware,
        enu_origin: ENUPose | None = None,
        relative_home: ENUPose | None = None,
        name: str = "guided_plan",
        wp_margin: float | None = None,
        navigation_speed: float = 5,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
        land: bool = True,
    ) -> Self:
        """Create GuidedPlan from relative path."""
        if enu_origin is None:
            enu_origin = ENUPose(0, 0, 0, 0)
        if relative_home is None:
            relative_home = ENUPose(0, 0, 0, 0)
        wps = ENUPose.resolve_path(enu_origin, relative_home, relative_path)
        return cls(
            name=name,
            wps=wps,
            wp_margin=wp_margin,
            navigation_speed=navigation_speed,
            land_bearing=land_bearing,
            autoland_alt=autoland_alt,
            autoland_wp_dist=autoland_wp_dist,
            land=land,
            firmware=firmware,
        )

    @classmethod
    def square_traj(
        cls,
        side_len: float,
        alt: float,
        firmware: Firmware,
        name: str = "guided_square_plan",
        enu_origin: ENUPose | None = None,
        relative_home: ENUPose | None = None,
        clockwise: bool = True,
        wp_margin: float | None = None,
        navigation_speed: float = 5,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
        land: bool = True,
    ) -> Self:
        """Create a square guided plan."""
        if enu_origin is None:
            enu_origin = ENUPose(0, 0, 0, 0)
        if relative_home is None:
            relative_home = ENUPose(0, 0, 0, 0)
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
            land_bearing=land_bearing,
            autoland_alt=autoland_alt,
            autoland_wp_dist=autoland_wp_dist,
            land=land,
            firmware=firmware,
        )
