"""
Firmware-aware navigation speed change via MAVLink SET_PARAM.

ArduCopter: WPNAV_SPEED (cm/s) — controls waypoint navigation speed.
ArduPlane:  AIRSPEED_CRUISE (m/s) — TECS cruise airspeed target.
            Requires ARSPD_USE=1 (airspeed sensor enabled) to take effect;
            without it TECS ignores this value and uses TRIM_THROTTLE instead.
"""

from __future__ import annotations

from typing import Literal

from simulator.helpers.ardupilot.enums import AirSpeed, WPNav
from simulator.helpers.connections.mavlink.enums import ParamType
from simulator.planner.action import Action
from simulator.planner.step import Step


class SetSpeed(Step):
    """Set the vehicle's navigation speed parameter."""

    def __init__(
        self,
        name: str,
        speed: float,
        firmware: Literal["ArduPlane", "ArduCopter"],
    ) -> None:
        super().__init__(name)
        self.param_id: bytes
        match firmware:
            case "ArduPlane":
                self.param_id = AirSpeed.CRUISE
                self.param_value = speed  # m/s
            case "ArduCopter":
                self.param_id = WPNav.SPEED
                self.param_value = speed * 100  # cm/s
        self.expected_id = self.param_id.decode("ascii")

    def exec_fn(self) -> None:
        # Drain stale PARAM_VALUE so check_fn only sees replies to this PARAM_SET.
        while self.mav_manager.state.wait_for("PARAM_VALUE", timeout=0) is not None:
            pass

        msg = self.conn.mav.param_set_encode(
            self.conn.target_system,
            self.conn.target_component,
            self.param_id,
            self.param_value,
            ParamType.REAL32,
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        # Scan queued PARAM_VALUE for ours; discard the rest.
        wait = self.mav_manager.state.wait_for
        while (msg := wait("PARAM_VALUE", timeout=0)) is not None:
            if isinstance(msg.param_id, (bytes, bytearray)):
                param_id = msg.param_id.decode("ascii", errors="replace")
            param_id = msg.param_id.rstrip("\x00")
            value_ok = abs(msg.param_value - self.param_value) < 0.01
            if param_id == self.expected_id and value_ok:
                return True
        return False


def make_change_nav_speed(
    speed: float,
    firmware: Literal["ArduPlane", "ArduCopter"] = "ArduCopter",
) -> "Action[Step]":
    """Return an Action that sets the vehicle's navigation speed."""
    name = Action.Names.CHANGE_NAVSPEED
    action: Action[Step] = Action[Step](name=name, emoji=name.emoji)
    action.add(
        SetSpeed(name=f"Set speed to {speed:.2f} m/s", speed=speed, firmware=firmware)
    )
    return action
