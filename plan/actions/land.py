"""
Defines a LAND action with execution and landing check using MAVLink commands.
"""

from helpers.change_coordinates import Position
from helpers.mavlink import MAVCommand, MAVConnection, ask_msg, stop_msg
from plan.actions.navegation import get_local_position
from plan.core import Action, ActionNames, Step


def make_land(final_wp: Position):
    """Create a LAND Action with execution and check steps."""
    example_action = Action(name=ActionNames.LAND, emoji="🛬")
    example_action.add_step(
        Step(
            "land",
            check_fn=check_land,
            exec_fn=exec_land,
            target_pos=final_wp,
            onair=True,
        )
    )
    return example_action


def exec_land(
    conn: MAVConnection,
    ask_pos_interval: int = 100_000,
    ask_land_interval: int = 100_000,
):
    """Send a MAVLink command to initiate landing."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MAVCommand.LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ask_msg(conn, MAVCommand.EXT_STATE, interval=ask_land_interval)
    ask_msg(conn, MAVCommand.LOCAL_POSITION_NED_ID, interval=ask_pos_interval)


def check_land(conn: MAVConnection, verbose: int):
    """Check if the UAV has landed using EXTENDED_SYS_STATE."""
    # parameter 4 is confirmation(it may be increased)
    msg = conn.recv_match(type="EXTENDED_SYS_STATE")
    current_pos = get_local_position(conn)
    if current_pos is not None and verbose > 1:
        print(f"Vehicle {conn.target_system}: 🛬 Altitute: {current_pos[2]:.2f} m")
    on_ground = bool(msg and msg.landed_state == MAVCommand.ON_GROUND)
    if on_ground:
        stop_msg(conn, MAVCommand.EXT_STATE)
    return on_ground, current_pos
