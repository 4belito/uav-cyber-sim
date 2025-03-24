# actions/pre_arm.py

from pymavlink import mavutil
from plan.core import Step, Action, StepFailed  # assuming these are in your core module

# Required EKF and sensor flags
EKF_FLAGS = {
    "ATTITUDE": mavutil.mavlink.EKF_ATTITUDE,
    "VELOCITY_HORIZ": mavutil.mavlink.EKF_VELOCITY_HORIZ,
    "POS_HORIZ_ABS": mavutil.mavlink.EKF_POS_HORIZ_ABS,
    "POS_VERT_ABS": mavutil.mavlink.EKF_POS_VERT_ABS,
}

REQUIRED_SENSORS = {
    "3D_GYRO": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
    "3D_ACCEL": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
    "3D_MAG": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
    "ABS_PRESSURE": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    "GPS": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS,
}

# === CHECK FUNCTIONS ===


def check_disarmed(conn,blocking=False):
    msg = conn.recv_match(type="HEARTBEAT", blocking=blocking)
    if not msg:
        return False
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise StepFailed("UAV is already armed")
    return True

def check_ekf_status(conn,blocking=False):
    msg = conn.recv_match(type="EKF_STATUS_REPORT", blocking=blocking)
    if not msg:
        return False
    missing = [name for name, bit in EKF_FLAGS.items() if not msg.flags & bit]
    if missing:
        print(f"Vehicle {conn.target_system}: EKF is not ready — missing: {', '.join(missing)}")
        return False
    return True

def check_gps_status(conn,blocking=False):
    msg = conn.recv_match(type="GPS_RAW_INT", blocking=blocking)
    if not msg:
        return False
    if msg.fix_type < 3:
        raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
    return True

def check_sys_status(conn,blocking=False):
    msg = conn.recv_match(type="SYS_STATUS", blocking=blocking)
    if not msg:
        return False
    if msg.battery_remaining < 20:
        raise StepFailed(f"Battery too low ({msg.battery_remaining}%)")
    missing = [name for name, bit in REQUIRED_SENSORS.items() if not msg.onboard_control_sensors_health & bit]
    if missing:
        raise StepFailed(f"Missing or unhealthy sensors: {', '.join(missing)}")
    return True



def make_pre_arm():
    pre_arm = Action("Pre-Arm Check")    
    pre_arm.add(Step("Check disarmed",check_fn=check_disarmed))
    pre_arm.add(Step("Check EKF", check_fn=check_ekf_status))
    pre_arm.add(Step("Check GPS", check_fn=check_gps_status))
    pre_arm.add(Step("Check system",check_fn=check_sys_status))
    return pre_arm
