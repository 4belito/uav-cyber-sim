import numpy as np
from typing import Optional
from .core import Action

from plan.actions import (
    make_pre_arm,
    make_set_mode,
    make_arm,
    make_takeoff,
    make_land,
    make_change_nav_speed,
    make_path,
)


class State:
    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"


class PlanMode:
    DYNAMIC = "DYNAMIC"
    STATIC = "STATIC"


class Plan(Action):
    def __init__(
        self,
        name: str,
        emoji: str = "📋",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: Optional[np.ndarray] = None,
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
        super().__init__(name, emoji=emoji, curr_pos=np.zeros(3))

    @staticmethod
    def create_square_path(side_len: float = 10, alt: float = 5, clockwise=True):
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
        wps = cls.create_square_path(side_len, alt)
        return cls.basic(
            wps=wps,
            wp_margin=wp_margin,
            navegation_speed=navegation_speed,
            name="Square Trajectory",
        )

    @classmethod
    def basic(
        cls,
        wps: np.ndarray = np.array([[0, 0, 5]]),
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name: str = "basic",
        mode: PlanMode = PlanMode.STATIC,
        dynamic_wps: Optional[np.ndarray] = None,
        takeoof_alt=1,
    ):
        land_wp = wps[-1].copy()
        land_wp[2] = 0
        plan = cls(name=name, mode=mode, dynamic_wps=dynamic_wps, wp_margin=wp_margin)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoof_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))

        plan.add(make_land(final_wp=land_wp))
        return plan

    ## Improve this for no repeating code
    @classmethod
    def hover(
        cls,
        wps: np.ndarray = None,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name="hover",
        takeoff_alt=5,
    ):
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=takeoff_alt))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        return plan
