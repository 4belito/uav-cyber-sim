"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

from functools import partial

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import MsgID
from mavlink.util import ask_msg, stop_msg
from plan import Action, ActionNames
from plan.core import Step


def make_monitoring(items: list[int] = []) -> Action[Step]:
    """Monitor mission items."""
    monitoring = Action[Step](name=ActionNames.UPLOAD_MISSION, emoji="📤")
    for i in items:
        monitoring.add(
            Step(
                "check item",
                exec_fn=Step.noop_exec,
                check_fn=partial(check_item, seq=i),
                onair=False,
            )
        )
    monitoring.add(
        Step(
            "check item",
            exec_fn=Step.noop_exec,
            check_fn=check_endmission,
            onair=False,
        )
    )
    return monitoring


def check_item(
    conn: MAVConnection,
    verbose: int,
    seq: int,
) -> tuple[bool, None]:
    """Check if a item is reached."""
    msg = conn.recv_match(type="MISSION_ITEM_REACHED", blocking=True)
    if msg:
        if verbose and msg.seq == seq:
            print(f"Vehicle {conn.target_system}: ✴️ Reached waypoint: {msg.seq}")
            return True, None
    else:
        print(f"Vehicle {conn.target_system}: loss reached item {seq} message")
    return False, None


def check_endmission(
    conn: MAVConnection,
    verbose: int,
) -> tuple[bool, None]:
    """Check missioin completion."""
    msg = conn.recv_match(type="STATUSTEXT", blocking=True)
    if msg:
        text = msg.text.strip().lower()
        if "disarming" in text:
            print(f"Vehicle {conn.target_system}: 🏁 Mission completed")
            if verbose > 1:
                stop_msg(conn, msg_id=MsgID.GLOBAL_POSITION_INT)
            return True, None
    return False, None


def exec_monitoring(
    conn: MAVConnection,
    verbose: int,
) -> None:
    """Start monitoring the UAV by requesting periodic GLOBAL_POSITION_INT."""
    if verbose > 1:
        ask_msg(conn, verbose, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000)
