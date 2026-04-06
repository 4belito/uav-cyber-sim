"""Pursuit plan: continuously fly toward a target vehicle's Remote ID position."""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Self

from simulator.entities.riddata import RIDData
from simulator.helpers.connections.mavlink.enums import Frame, MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.helpers.coordinates import ENU
from simulator.planner.action import Action
from simulator.planner.actions import make_takeoff
from simulator.planner.plan import Plan, PlanSpec
from simulator.planner.step import Step

RIDGetter = Callable[[int], RIDData | None]

_TYPE_MASK = int(0b110111111000)  # position only, same as GoTo


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

    def _send_goto(self, target: ENU) -> None:
        gra_wp = self.origin.to_abs(target)
        go_msg = self.conn.mav.set_position_target_global_int_encode(
            10,
            self.conn.target_system,
            self.conn.target_component,
            Frame.GLOBAL_INT,
            _TYPE_MASK,
            *gra_wp.to_global_int_alt_in_meters(),
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )
        self.conn.mav.send(go_msg)

    def exec_fn(self) -> None:
        """Send initial GoTo if target RID is already available."""
        msg = ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT, interval=100_000)
        self.mav_manager.send(msg)
        if self._get_target is not None:
            rid = self._get_target(self.target_sysid)
            if rid is not None:
                self.target_pos = rid.enu_pos
                self._send_goto(rid.enu_pos)
                self._last_update = time.time()

    def check_fn(self) -> bool:
        """Refresh GoTo with the latest target position; never signals done."""
        now = time.time()
        if now - self._last_update >= self.update_interval:
            if self._get_target is not None:
                rid = self._get_target(self.target_sysid)
                if rid is not None:
                    self.target_pos = rid.enu_pos
                    self._send_goto(rid.enu_pos)
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
        speed: float = 3.0,
        takeoff_alt: float = 1.0,
        update_interval: float = 1.0,
    ) -> None:
        super().__init__(name=name)
        self.target_sysid = target_sysid

        self.extend(Plan.arm(navigation_speed=speed))
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
            },
        )

    def bind_rid_getter(self, fn: RIDGetter) -> None:
        """Inject the RID lookup function provided by the logic process."""
        self._pursue_step._get_target = fn

    @classmethod
    def from_spec(cls, **kwargs: Any) -> Self:
        return cls(
            name=kwargs["name"],
            target_sysid=kwargs["target_sysid"],
            speed=kwargs.get("speed", 3.0),
            takeoff_alt=kwargs.get("takeoff_alt", 1.0),
            update_interval=kwargs.get("update_interval", 1.0),
        )
