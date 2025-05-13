"""
Pre-arm safety checks for UAV operation.

This module defines individual checks and a combined pre-arm `Action` to ensure
the UAV is in a safe and ready state before arming. It verifies the following:

- The UAV is disarmed
- EKF system is properly initialized
- GPS fix is sufficient (3D or better)
- Battery level is acceptable
- Required sensors are healthy

The main entry point is `make_pre_arm()`, which returns an `Action` composed of
these checks in sequence.
"""

from functools import partial

from plan.core import Action, ActionNames, Step, StepFailed
from plan.mav_helpres import EKFFlags, MAVCommand, MAVConnection, RequiredSensors


def make_pre_arm():
    """Builds a pre-arm Action that validates safety and system readiness checks."""
    pre_arm = Action(name=ActionNames.PREARM, emoji="🔧")
    pre_arm.add_step(Step("Check disarmed", check_fn=check_disarmed, onair=False))
    pre_arm.add_step(
        Step(
            "Check EKF",
            check_fn=partial(check_ekf_status),
            onair=False,
        )
    )
    pre_arm.add_step(Step("Check GPS", check_fn=check_gps_status, onair=False))
    pre_arm.add_step(Step("Check system", check_fn=check_sys_status, onair=False))
    return pre_arm


# === CHECK FUNCTIONS ===
def check_disarmed(conn: MAVConnection, _verbose: int):
    """Fails if the UAV is currently armed."""
    msg = conn.recv_match(type="HEARTBEAT")
    if not msg:
        return False, None
    if msg.base_mode & MAVCommand.ARMED_FLAG:
        raise StepFailed("UAV is already armed")
    return True, None


def check_ekf_status(conn: MAVConnection, _verbose: int):
    """Checks whether all required EKF flags are set."""
    msg = conn.recv_match(type="EKF_STATUS_REPORT")
    if not msg:
        return False, None
    missing = [flag.name for flag in EKFFlags if not msg.flags & flag]
    if missing:
        return False, None
    return True, None


def check_gps_status(conn: MAVConnection, _verbose: int):
    """Fails if GPS fix is not 3D (fix_type < 3)."""
    msg = conn.recv_match(type="GPS_RAW_INT")
    if not msg:
        return False, None
    if msg.fix_type < 3:
        raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
    return True, None


def check_sys_status(conn: MAVConnection, _verbose: int):
    """Fails if battery is low or any required sensors are unhealthy."""
    msg = conn.recv_match(type="SYS_STATUS")
    if not msg:
        return False, None
    if msg.battery_remaining < 20:
        raise StepFailed(f"Battery too low ({msg.battery_remaining}%)")

    missing = [
        sensor.name
        for sensor in RequiredSensors
        if not msg.onboard_control_sensors_health & sensor
    ]

    if missing:
        raise StepFailed(f"Missing or unhealthy sensors: {', '.join(missing)}")
    return True, None
