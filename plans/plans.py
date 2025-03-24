import numpy as np

from plans.planner import Plan
from plans.actions.pre_arm import make_pre_arm
from plans.actions.change_mode import make_set_mode
from plans.actions.arm import make_arm
from plans.actions.take_off import make_takeoff
from plans.actions.navegation import make_path
from plans.actions.land import make_land
from plans.actions.change_parameter import make_change_nav_speed



def create_square_path(side_len:float = 10,alt:float=5):
    wps=np.array([(0, 0, alt), #takeoff point
        (0,side_len, alt),
        (side_len, side_len, alt),
        (side_len, 0, alt),
        (0, 0, alt)])
    return wps
    

def make_square_plan(side_len:float = 10,alt:float=5,wp_margin:float=0.5,navegation_speed:float=5):
    wps=create_square_path(side_len=side_len,alt=alt)
    return make_plan_from_wps(wps,alt=alt,wp_margin=wp_margin,navegation_speed=navegation_speed,name='Square Trajectory')


def make_plan_from_wps(wps:np.ndarray,alt:float=5,wp_margin:float=0.5,navegation_speed:float=5,name='Waypoints Trajectory'):
    plan=Plan(name)
    plan.add(make_pre_arm())
    if navegation_speed!=5:
        plan.add(make_change_nav_speed(speed=navegation_speed))
    plan.add(make_set_mode('GUIDED'))
    plan.add(make_arm())
    plan.add(make_takeoff(altitude=alt,wp_margin=wp_margin))
    plan.add(make_path(wps=wps,wp_margin=wp_margin))
    plan.add(make_land())
    return plan