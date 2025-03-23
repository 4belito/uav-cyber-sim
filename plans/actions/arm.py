
from pymavlink import mavutil
from plans.planner import Step, Action, StepFailed 


class MAVCommand:
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    REQUEST_MESSAGE = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    LOITER_UNLIMITED = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM

def exec_arm(conn: mavutil.mavlink_connection, blocking: bool = False) -> None:
    """Send ARM command to the UAV."""
    print("🛰️ Sending ARM command...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0  # 1 = arm
    )

def check_arm(conn: mavutil.mavlink_connection, blocking: bool = False) -> bool:
    """Check if the UAV is armed via HEARTBEAT."""
    msg = conn.recv_match(type="HEARTBEAT", blocking=blocking, timeout=2)
    if not msg:
        return False  # Still waiting
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    if not is_armed:
        print("UAV is not armed yet.")
        return False
    return True

def make_arm():
    arm = Action("Arm")
    arm.add(Step("arm",check_fn=check_arm,exec_fn=exec_arm))
    return arm