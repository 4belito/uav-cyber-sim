import numpy as np

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


class Plan(Action):
    def __init__(self, name: str, verbose: bool = False) -> None:
        super().__init__(name, curr_pos=np.zeros(3), verbose=verbose)

    @staticmethod
    def create_square_path(side_len: float = 10, alt: float = 5):
        return np.array(
            [
                (0, 0, alt),
                (0, side_len, alt),
                (side_len, side_len, alt),
                (side_len, 0, alt),
                (0, 0, alt),
            ]
        )

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
            alt=alt,
            wp_margin=wp_margin,
            navegation_speed=navegation_speed,
            name="Square Trajectory",
        )

    @classmethod
    def basic(
        cls,
        wps: np.ndarray = None,
        alt: float = 5,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name="basic",
    ):
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=alt, wp_margin=wp_margin))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        if wps is None:
            land_wp = np.zeros(3)
        else:
            land_wp = wps[-1].copy()
            land_wp[2] = 0
        plan.add(make_land(wp=np.zeros(3)))
        return plan

    ## Improve this for no repeating code
    @classmethod
    def hover(
        cls,
        wps: np.ndarray = None,
        alt: float = 5,
        wp_margin: float = 0.5,
        navegation_speed: float = 5,
        name="hover",
    ):
        plan = cls(name)
        plan.add(make_pre_arm())
        plan.add(make_set_mode("GUIDED"))
        if navegation_speed != 5:
            plan.add(make_change_nav_speed(speed=navegation_speed))
        plan.add(make_arm())
        plan.add(make_takeoff(altitude=alt, wp_margin=wp_margin))
        plan.add(make_path(wps=wps, wp_margin=wp_margin))
        return plan
