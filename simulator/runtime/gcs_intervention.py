"""Run a GCS intervention plan against a vehicle over its command channel."""

from __future__ import annotations

import logging
import time
from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink

from simulator.config import DATA_PATH
from simulator.entities.intervention import Intervention, TriggerContext
from simulator.helpers.ardupilot.firmware import auto_mode
from simulator.helpers.connections import MAVConnection
from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.helpers.coordinates import ENU, GRA
from simulator.helpers.logging.data_logger import DataLogger
from simulator.planner.actions import make_set_mode
from simulator.planner.step import State
from simulator.runtime.vehicle.mav_manager import MAVLinkManager

# How often the vehicle is asked to report its position, in microseconds. The
# proximity guard needs a steady position fix even before it engages.
_POSITION_INTERVAL_US = 200_000


class InterventionRunner:
    """
    Hold a vehicle under GCS control while an intervention's trigger holds.

    Reuses the Plan/Action/Step machinery on the GCS side: the plan is bound to a
    `MAVLinkManager` whose connection is the vehicle's command channel — sends
    reach SITL through the vehicle's GCS command forwarder — and whose telemetry
    state is fed from the GCS monitor loop via `feed`.

    The trigger is re-evaluated on every `tick`, so this is a guard rather than a
    one-shot: while it holds, each tick advances the intervention plan one step;
    when it stops holding, the GCS switches the vehicle back to AUTO (its paused
    mission resumes) and re-arms, ready to engage again. A trigger built only from
    the monotone `seq`/`after` conditions never stops holding, which is the
    original take-over-and-keep-it behaviour.
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
        # Handing control back is just a mode switch: ArduPilot resumes the
        # mission from the item it was on when the GCS took over.
        self._release = make_set_mode(auto_mode(intervention.firmware))
        self.plan.bind(self.gra_origin, self._mav)
        self._release.bind(self.gra_origin, self._mav)
        # The guard reads position every tick, so keep it streaming rather than
        # relying on whatever rate happens to be active.
        self._mav.send(
            ask_msg(cmd_conn, MsgID.GLOBAL_POSITION_INT, interval=_POSITION_INTERVAL_US)
        )
        self._t0 = time.monotonic()
        self._current_seq: int | None = None
        # When the trigger's seq point was first reached (for `dwell`).
        self._seq_reached_at: float | None = None
        self._engaged = False

    def feed(self, msg: mavlink.MAVLink_message) -> None:
        """Update the telemetry cache and track the latest mission sequence."""
        self._mav.state.update(msg)
        if msg.get_type() == "MISSION_CURRENT":
            mission_current = cast(mavlink.MAVLink_mission_current_message, msg)
            self._current_seq = int(mission_current.seq)

    def tick(self) -> None:
        """Engage, drive, or release the intervention based on the trigger."""
        if self._holds():
            if not self._engaged:
                logging.info(f"GCS intervention: taking over vehicle {self.sysid}")
                self.plan.reset()
                self._engaged = True
            self.plan.act()
        elif self._engaged:
            # Trigger cleared: switch back to AUTO, then re-arm for next time.
            self._release.act()
            if self._release.state == State.DONE:
                logging.info(f"GCS intervention: released vehicle {self.sysid} to AUTO")
                self._release.reset()
                self.plan.reset()
                self._engaged = False

    def _holds(self) -> bool:
        """Whether the trigger says the GCS should be in control right now."""
        now = time.monotonic()
        # Stamp when the seq point is first reached, so `dwell` can measure from
        # there rather than from the start of monitoring.
        trigger_seq = getattr(self.trigger, "seq", None)
        if (
            self._seq_reached_at is None
            and trigger_seq is not None
            and self._current_seq is not None
            and self._current_seq >= trigger_seq
        ):
            self._seq_reached_at = now
        seq_elapsed = (
            None if self._seq_reached_at is None else now - self._seq_reached_at
        )
        return self.trigger.holds(
            TriggerContext(
                current_seq=self._current_seq,
                elapsed=now - self._t0,
                seq_elapsed=seq_elapsed,
                position=self._position(),
                engaged=self._engaged,
            )
        )

    def _position(self) -> ENU | None:
        """Latest vehicle position in the run's ENU frame, or None if unknown."""
        msg = self._mav.state.get("GLOBAL_POSITION_INT")
        if msg is None:
            return None
        return self.gra_origin.to_rel(GRA.from_global_int(msg.lat, msg.lon, msg.alt))

    @property
    def engaged(self) -> bool:
        """Whether the GCS currently holds control of the vehicle."""
        return self._engaged

    @property
    def done(self) -> bool:
        """Whether the intervention is engaged and its plan has finished."""
        return self._engaged and self.plan.state == State.DONE
