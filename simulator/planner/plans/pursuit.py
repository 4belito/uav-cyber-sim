"""Pursuit plan: continuously fly toward a target vehicle's Remote ID position."""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Self

from simulator.entities.riddata import RIDData
from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.planner.action import Action
from simulator.planner.actions import make_takeoff
from simulator.planner.plan import Plan, PlanSpec
from simulator.planner.step import Step

if TYPE_CHECKING:
    from simulator.config import Firmware

RIDGetter = Callable[[int], RIDData | None]


class PursueStep(Step):
    """
    Looping step that continuously commands the vehicle toward a target's
    last known Remote ID position. Never completes — the pursuer keeps
    flying until the simulation is stopped.
    """

    def __init__(
        self,
        name: str,
        target_sysid: int,
        update_interval: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.target_sysid = target_sysid
        self.update_interval = update_interval
        self._get_target: RIDGetter | None = None
        self._last_update: float = 0.0

    def set_target_getter(self, fn: RIDGetter) -> None:
        """Set the RID lookup function used by this step."""
        self._get_target = fn

    def exec_fn(self) -> None:
        """Send initial GoTo if target RID is already available."""
        msg = ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT, interval=100_000)
        self.mav_manager.send(msg)
        if self._get_target is not None:
            rid = self._get_target(self.target_sysid)
            if rid is not None:
                self.target_pos = rid.enu_pos
                self.send_position_target(rid.enu_pos)
                self._last_update = self.mav_manager.state.sim_time_s() or 0.0

    def check_fn(self) -> bool:
        """Refresh GoTo with the latest target position; never signals done."""
        # Sim clock, so `update_interval` is sim seconds regardless of `speedup`.
        now = self.mav_manager.state.sim_time_s()
        if now is None:
            return False
        update_needed = now - self._last_update >= self.update_interval
        if update_needed and self._get_target is not None:
            rid = self._get_target(self.target_sysid)
            if rid is not None:
                self.target_pos = rid.enu_pos
                self.send_position_target(rid.enu_pos)
                logging.info(
                    f"🎯 Pursuer {self.sysid}: targeting sysid={self.target_sysid}"
                    f" at {rid.enu_pos}"
                )
            else:
                logging.debug(
                    f"🎯 Pursuer {self.sysid}: no RID yet for sysid={self.target_sysid}"
                )
            self._last_update = now
        return False  # never completes


@Plan.register("PursuitPlan")
class PursuitPlan(Plan):
    """
    Arms, takes off, then pursues a target vehicle using its Remote ID
    position broadcasts.
    """

    def __init__(
        self,
        name: str,
        target_sysid: int,
        firmware: Firmware,
        speed: float = 3.0,
        takeoff_alt: float = 1.0,
        update_interval: float = 1.0,
    ) -> None:
        super().__init__(name=name)
        self.target_sysid = target_sysid

        self.extend(Plan.arm(firmware=firmware, navigation_speed=speed))
        self.add(make_takeoff(altitude=takeoff_alt))

        pursue_action = Action[Step](
            name=Action.Names.FLY, emoji=Action.Names.FLY.emoji
        )
        self._pursue_step = PursueStep(
            name=f"pursue sysid={target_sysid}",
            target_sysid=target_sysid,
            update_interval=update_interval,
        )
        pursue_action.add(self._pursue_step)
        self.add(pursue_action)

        self._spec = PlanSpec(
            plan_class="PursuitPlan",
            kwargs={
                "name": name,
                "target_sysid": target_sysid,
                "speed": speed,
                "takeoff_alt": takeoff_alt,
                "update_interval": update_interval,
                "firmware": firmware,
            },
        )

    def bind_rid_getter(self, fn: RIDGetter) -> None:
        """Inject the RID lookup function provided by the logic process."""
        self._pursue_step.set_target_getter(fn)

    @classmethod
    def from_spec(cls, **kwargs: Any) -> Self:
        """Build PursuitPlan from JSON-serializable arguments."""
        return cls(
            name=kwargs["name"],
            target_sysid=kwargs["target_sysid"],
            speed=kwargs.get("speed", 3.0),
            takeoff_alt=kwargs.get("takeoff_alt", 1.0),
            update_interval=kwargs.get("update_interval", 1.0),
            firmware=kwargs["firmware"],
        )
