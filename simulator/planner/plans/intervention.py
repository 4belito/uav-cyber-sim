"""Intervention plan: a guided sequence a GCS applies to an already-flying vehicle."""

from __future__ import annotations

from typing import Any, Self

from simulator.config import Firmware
from simulator.helpers.ardupilot.firmware import guided_mode
from simulator.helpers.coordinates import ENU, ENUPose, ENUs
from simulator.helpers.math import enu_bearing
from simulator.planner.actions import (
    make_hold,
    make_land,
    make_path,
    make_set_mode,
)
from simulator.planner.plan import Plan, PlanSpec


@Plan.register("InterventionPlan")
class InterventionPlan(Plan):
    """
    A guided plan a GCS runs against a vehicle that is already airborne.

    Unlike `GuidedPlan` it has **no arm/pre-arm/takeoff** — the target is already
    flying its own mission when the GCS takes over. It switches the vehicle to
    GUIDED, flies it through `wps` (all treated as go-to targets, none dropped),
    then holds (default) or lands. Every waypoint is an ENU relative to the run
    origin; the go-to steps convert to geodetic at `bind` time.
    """

    def __init__(
        self,
        name: str,
        wps: ENUs,
        firmware: Firmware,
        wp_margin: float | None = None,
        land: bool = False,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
    ) -> None:
        super().__init__(name=name)
        self.add(make_set_mode(guided_mode(firmware)))
        # `stop_asking_pos=False`: arriving at a waypoint must not switch off the
        # position stream, which the GCS proximity guard reads every tick.
        self.add(
            make_path(
                wps=wps,
                wp_margin=wp_margin,
                stop_asking_pos=False,
                firmware=firmware,
            )
        )
        if not land:
            self.add(make_hold())
        elif firmware == "ArduPlane":
            if land_bearing is None and len(wps) >= 2:
                land_bearing = enu_bearing(wps[-2], wps[-1])
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
            plan_class="InterventionPlan",
            kwargs={
                "name": name,
                "wps": wps,
                "wp_margin": wp_margin,
                "land": land,
                "land_bearing": land_bearing,
                "autoland_alt": autoland_alt,
                "autoland_wp_dist": autoland_wp_dist,
                "firmware": firmware,
            },
        )

    @classmethod
    def from_spec(cls, **kwargs: Any) -> InterventionPlan:
        """Rebuild an InterventionPlan from its serialized spec kwargs."""
        missing = {"name", "wps", "firmware"} - kwargs.keys()
        if missing:
            raise ValueError(f"Missing spec fields: {sorted(missing)}")
        enu_wps: ENUs = [ENU(*wp) for wp in kwargs["wps"]]
        return cls(
            name=kwargs["name"],
            wps=enu_wps,
            firmware=kwargs["firmware"],
            wp_margin=kwargs.get("wp_margin"),
            land=kwargs.get("land", False),
            land_bearing=kwargs.get("land_bearing"),
            autoland_alt=kwargs.get("autoland_alt"),
            autoland_wp_dist=kwargs.get("autoland_wp_dist"),
        )

    @classmethod
    def from_relative_path(
        cls,
        relative_path: ENUs,
        firmware: Firmware,
        enu_origin: ENUPose | None = None,
        relative_home: ENUPose | None = None,
        name: str = "intervention_plan",
        wp_margin: float | None = None,
        land: bool = False,
        land_bearing: float | None = None,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
    ) -> Self:
        """Create an InterventionPlan from a path given relative to an ENU origin."""
        if enu_origin is None:
            enu_origin = ENUPose(0, 0, 0, 0)
        if relative_home is None:
            relative_home = ENUPose(0, 0, 0, 0)
        wps = ENUPose.resolve_path(enu_origin, relative_home, relative_path)
        return cls(
            name=name,
            wps=wps,
            firmware=firmware,
            wp_margin=wp_margin,
            land=land,
            land_bearing=land_bearing,
            autoland_alt=autoland_alt,
            autoland_wp_dist=autoland_wp_dist,
        )
