"""
Defines logic for creating and verifying a mission step that changes
the UAV's WPNAV_SPEED parameter via MAVLink SET_PARAM commands.

Includes:
- `make_change_nav_speed()`: returns an Action to set speed.
- `exec_set_nav_speed()`: sends the SET_PARAM command.
- `check_set_nav_speed()`: confirms the parameter was applied.
"""

from helpers.ardupilot.enums import WPNav
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import ParamType
from plan.core import Action, ActionNames, Step


def make_change_nav_speed(speed: float) -> Action[Step]:
    """Return an Action that changes the UAV's WPNAV_SPEED."""
    action = Action[Step](name=ActionNames.CHANGE_NAVSPEED, emoji="🎚️")

    class SetSpeed(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Send a SET_PARAM command to change WPNAV_SPEED (navigation speed)."""
            speed_cmps = speed * 100  # ArduPilot uses cm/s
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                WPNav.SPEED,
                speed_cmps,
                ParamType.REAL32,
            )

        def check_fn(self, conn: MAVConnection) -> bool:
            """Check whether the WPNAV_SPEED parameter has been updated."""
            msg = conn.recv_match(type="PARAM_VALUE")
            if not msg:
                return False

            # check this decode(added to avoid warning)
            expected_id = WPNav.SPEED.decode("ascii")
            speed_cmps = speed * 100

            return msg.param_id == expected_id and msg.param_value == speed_cmps

    step = SetSpeed(name=f"Set speed to {speed:.2f} m/s")
    action.add(step)
    return action
