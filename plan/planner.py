import numpy as np

from plan.core import Action

from plan.actions.pre_arm import make_pre_arm
from plan.actions.change_mode import make_set_mode
from plan.actions.arm import make_arm
from plan.actions.take_off import make_takeoff
from plan.actions.land import make_land
from plan.actions.change_parameter import make_change_nav_speed
from plan.actions.navegation import make_path


class State:
    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"

class Plan(Action):
    def __init__(self, name: str) -> None:
        super().__init__(name)

    @staticmethod
    def create_square_path(side_len: float = 10, alt: float = 5):
        return np.array([
            (0, 0, alt),
            (0, side_len, alt),
            (side_len, side_len, alt),
            (side_len, 0, alt),
            (0, 0, alt)
        ])

    @classmethod
    def square(cls, side_len: float = 10, alt: float = 5, wp_margin: float = 0.5, navegation_speed: float = 5,verbose:int=0):
        wps = cls.create_square_path(side_len, alt)
        return cls.basic(wps=wps, alt=alt, wp_margin=wp_margin, navegation_speed=navegation_speed, name="Square Trajectory",verbose=verbose)

    @classmethod
    def basic(cls,wps:np.ndarray=None,alt:float=5,wp_margin:float=0.5,navegation_speed:float=5,name='basic',verbose:int=0):
        plan=cls(name)
        plan.add(make_pre_arm(verbose=verbose))
        plan.add(make_set_mode('GUIDED',verbose=verbose))
        if navegation_speed!=5:
            plan.add(make_change_nav_speed(speed=navegation_speed,verbose=verbose))
        
        plan.add(make_arm(verbose=verbose))
        plan.add(make_takeoff(altitude=alt,wp_margin=wp_margin,verbose=verbose))
        plan.add(make_path(wps=wps,wp_margin=wp_margin,verbose=verbose))
        plan.add(make_land(verbose=verbose))
        return plan
    
    ## Improve this for no repeating code
    @classmethod
    def hover(cls,wps:np.ndarray=None,alt:float=5,wp_margin:float=0.5,navegation_speed:float=5,name='hover',verbose:int=0):
        plan=cls(name)
        plan.add(make_pre_arm(verbose=verbose))
        
        if navegation_speed!=5:
            plan.add(make_change_nav_speed(speed=navegation_speed,verbose=verbose))        
        plan.add(make_arm(verbose=verbose))
        plan.add(make_set_mode('GUIDED',verbose=verbose))
        plan.add(make_takeoff(altitude=alt,wp_margin=wp_margin,verbose=verbose))
        plan.add(make_path(wps=wps,wp_margin=wp_margin,verbose=verbose))
        return plan