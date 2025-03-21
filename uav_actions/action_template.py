
from pymavlink import mavutil
from mission_flow import Step, Action, StepFailed 

def step(self,conn,blocking=False):
    pass





def make_arm():
    check_noarmed_step = Step("Check disarmed",check_fn=step)
    check_ekf_status_step = Step("Check EKF", check_fn=step)


    pre_arm = Action("Pre-Arm Check")
    pre_arm.add_step(check_noarmed_step)
    pre_arm.add_step(check_ekf_status_step)

    return pre_arm