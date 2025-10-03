"""Custom MAVLink commands."""

from enum import IntEnum


class CmdCustom(IntEnum):
    """official MAV_CMD values generally range from 0 to ~2999."""

    PLAN_DONE = 3000  # Custom command to mark end of plan
