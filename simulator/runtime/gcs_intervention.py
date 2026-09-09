"""Run a GCS intervention plan against a vehicle over its command channel."""

from __future__ import annotations

import logging
import time

import pymavlink.dialects.v20.ardupilotmega as mavlink

from simulator.config import DATA_PATH
from simulator.entities.intervention import Intervention
from simulator.helpers.connections import MAVConnection
from simulator.helpers.coordinates import GRA
from simulator.helpers.logging.data_logger import DataLogger
from simulator.planner.step import State
from simulator.runtime.vehicle.mav_manager import MAVLinkManager


class InterventionRunner:
    """
    Fire a GCS intervention plan when its trigger is met and tick it to completion.

    Reuses the Plan/Action/Step machinery on the GCS side: the plan is bound to a
    `MAVLinkManager` whose connection is the vehicle's command channel — sends
    reach SITL through the vehicle's GCS command forwarder — and whose telemetry
    state is fed from the GCS monitor loop via `feed`. Once the trigger fires, each
    `tick` advances the plan one step.
    """

    def __init__(
        self,
        sysid: int,
        intervention: Intervention,
        cmd_conn: MAVConnection,
        gra_origin: GRA,
    ) -> None:
        self.sysid = sysid
        self.trigger = intervention.trigger
        self.plan = intervention.plan
        self.gra_origin = gra_origin
        # Commands are addressed to this vehicle's autopilot.
        cmd_conn.target_system = sysid
        cmd_conn.target_component = 1
        self._mav = MAVLinkManager(
            conn=cmd_conn,
            data_logger=DataLogger(path=DATA_PATH / "gcs_cmd", sysid=sysid),
        )
        self._t0 = time.monotonic()
        self._current_seq: int | None = None
        # When the trigger's seq point was first reached (for `dwell`).
        self._seq_reached_at: float | None = None
        self._armed = False

    def feed(self, msg: mavlink.MAVLink_message) -> None:
        """Update the telemetry cache and track the latest mission sequence."""
        self._mav.state.update(msg)
        if msg.get_type() == "MISSION_CURRENT":
            self._current_seq = int(msg.seq)

    def tick(self) -> None:
        """Advance the intervention: arm on the trigger, then one plan step per call."""
        if self.done:
            return
        if not self._armed:
            now = time.monotonic()
            # Stamp when the seq point is first reached, so `dwell` can measure
            # from there rather than from the start of monitoring.
            if (
                self._seq_reached_at is None
                and self.trigger.seq is not None
                and self._current_seq is not None
                and self._current_seq >= self.trigger.seq
            ):
                self._seq_reached_at = now
            seq_elapsed = (
                None if self._seq_reached_at is None else now - self._seq_reached_at
            )
            if not self.trigger.ready(self._current_seq, now - self._t0, seq_elapsed):
                return
            logging.info(f"GCS intervention: taking over vehicle {self.sysid}")
            self.plan.bind(self.gra_origin, self._mav)
            self._armed = True
        self.plan.act()

    @property
    def done(self) -> bool:
        """Whether the intervention fired and its plan has finished."""
        return self._armed and self.plan.state == State.DONE
