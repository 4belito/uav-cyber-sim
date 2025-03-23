import numpy as np

from plans.planner import Plan
from plans.actions.pre_arm import make_pre_arm
from plans.actions.change_mode import make_set_guided_mode
from plans.actions.arm import make_arm
from plans.actions.take_off import make_takeoff
from plans.actions.navegation import make_path
from plans.actions.land import make_land





def make_square(side_len:float = 10,alt:float=5,wp_margin:float=0.5):
    wps=np.array([(0, 0, alt), #takeoff point
        (0,side_len, alt),
        (side_len, side_len, alt),
        (side_len, -side_len, alt),
        (0, -side_len, alt),
        (0, 0, alt)])

    plan=Plan('Square Trajectory')
    plan.add(make_pre_arm())
    plan.add(make_set_guided_mode())
    plan.add(make_arm())
    plan.add(make_takeoff(altitude=alt,wp_margin=wp_margin))
    plan.add(make_path(wps=wps,wp_margin=wp_margin))
    plan.add(make_land())
    return plan
