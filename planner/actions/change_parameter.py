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
from planner.action import Action
from planner.step import Step


class SetSpeed(Step):
    """Step to set the UAV's WPNAV_SPEED parameter."""

    def __init__(self, name: str, speed: float) -> None:
        super().__init__(name)
        self.speed = speed

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send a SET_PARAM command to change WPNAV_SPEED (navigation speed)."""
        speed_cmps = self.speed * 100  # ArduPilot uses cm/s
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
        speed_cmps = self.speed * 100

        return msg.param_id == expected_id and msg.param_value == speed_cmps


def make_change_nav_speed(speed: float) -> Action[Step]:
    """Return an Action that changes the UAV's WPNAV_SPEED."""
    name = Action.Names.CHANGE_NAVSPEED
    action = Action[Step](name=name, emoji=name.emoji)
    step = SetSpeed(name=f"Set speed to {speed:.2f} m/s", speed=speed)
    action.add(step)
    return action
