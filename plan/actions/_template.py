
from pymavlink import mavutil
from plan.core import Step, Action
from functools import partial

def exec_step(self,conn,blocking=False,extra_arg=0,verbose:int=0):
    pass


def check_step(self,conn,blocking=False,extra_arg=0,verbose:int=0):
    pass



def make_action(verbose:int=0):
    example_action = Action("Example_Action")
    example_action.add(Step("step_1",check_fn=partial(check_step,extra_arg=1),exec_fn=partial(exec_step,extra_arg=-1)))
    example_action.add(Step("step_2", check_fn=partial(check_step,extra_arg=2)))
    return example_action


