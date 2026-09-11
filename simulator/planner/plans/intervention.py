"""Intervention plan: a guided sequence a GCS applies to an already-flying vehicle."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Self, cast

from simulator.helpers.ardupilot.firmware import auto_mode, guided_mode
from simulator.helpers.coordinates import ENU, ENUPose, ENUs
from simulator.helpers.math import enu_bearing
from simulator.planner.actions import (
    make_hold,
    make_land,
    make_path,
    make_set_mode,
)
from simulator.planner.plan import Plan, PlanSpec

if TYPE_CHECKING:
    from simulator.config import Firmware
    from simulator.helpers.connections.mavlink.enums import CopterMode, PlaneMode
    from simulator.planner.actions.change_mode import SwitchMode


@Plan.register("InterventionPlan")
class InterventionPlan(Plan):
    """
    A guided plan a GCS runs against a vehicle that is already airborne.

    Unlike `GuidedPlan` it has **no arm/pre-arm/takeoff** — the target is already
    flying its own mission when the GCS takes over. It switches the vehicle to
    GUIDED, flies it through `wps` (all treated as go-to targets, none dropped),
    then — depending on the tail option — holds position (default), lands
    (`land=True`), or hands the vehicle back to its own mission (`resume=True`:
    switch mode so ArduPilot continues from the paused item, then idle). With
    `resume` the intervention releases control even under a monotone
    `MissionTrigger`, because the release is part of the plan rather than the
    trigger. The resume mode defaults to firmware AUTO but `c`
    lets `InterventionRunner` override it with the vehicle's actual
    pre-takeover mode, so a GUIDED-flown target resumes to GUIDED rather than
    being forced into AUTO. Every waypoint is an ENU relative to the run
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
        resume: bool = False,
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
        self._resume_step: SwitchMode | None = None
        if land:
            if firmware == "ArduPlane":
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
        elif resume:
            # Hand the mission back: switching mode resumes from where control
            # was taken. Defaults to AUTO; kept as a step reference so
            # `set_resume_mode` can retarget it to the vehicle's actual
            # pre-takeover mode. The trailing hold keeps the plan from completing.
            resume_action = make_set_mode(auto_mode(firmware))
            self._resume_step = cast("SwitchMode", resume_action.steps[0])
            self.add(resume_action)
            self.add(make_hold())
        else:
            self.add(make_hold())

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
                "resume": resume,
                "firmware": firmware,
            },
        )

    def set_resume_mode(self, mode: CopterMode | PlaneMode) -> None:
        """
        Override the `resume=True` tail's target mode (default: firmware AUTO).

        `InterventionRunner` calls this at engage time with the vehicle's actual
        pre-takeover mode, so a GUIDED-flown target resumes to GUIDED instead of
        being forced into AUTO. No-op if this plan wasn't built with `resume=True`.
        """
        if self._resume_step is not None:
            self._resume_step.set_mode(mode)

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
            resume=kwargs.get("resume", False),
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
        resume: bool = False,
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
            resume=resume,
        )
