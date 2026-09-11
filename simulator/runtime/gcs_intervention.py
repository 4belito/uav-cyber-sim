"""Run a GCS intervention plan against a vehicle over its command channel."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, cast

from simulator.config import DATA_PATH
from simulator.entities.intervention import Intervention, TriggerContext
from simulator.helpers.ardupilot.firmware import auto_mode
from simulator.helpers.connections.mavlink.enums import CopterMode, MsgID, PlaneMode
from simulator.helpers.connections.mavlink.streams import ask_msg
from simulator.helpers.coordinates import ENU, GRA
from simulator.helpers.logging.data_logger import DataLogger
from simulator.planner.actions import make_set_mode
from simulator.planner.step import State
from simulator.runtime.vehicle.mav_manager import MAVLinkManager

if TYPE_CHECKING:
    import pymavlink.dialects.v20.ardupilotmega as mavlink

    from simulator.config import Firmware
    from simulator.helpers.connections import MAVConnection
    from simulator.planner.action import Action
    from simulator.planner.step import Step

# Position-report interval (µs): the proximity guard needs a fix before it engages.
_POSITION_INTERVAL_US = 200_000


class InterventionRunner:
    """
    Hold a vehicle under control while an intervention's trigger holds.

    Reuses the Plan/Action/Step machinery over a command channel to the vehicle;
    the caller feeds telemetry via `feed` and calls `tick`. The GCS drives it
    from its monitor loop, `InterventionStrategy` from the man-in-the-middle
    (`source` only changes the log wording).

    Each `tick` re-checks the trigger: while it holds, advance the plan a step;
    when it clears, restore the vehicle to **the flight mode it was in when the
    runner engaged** (AUTO for an `AutoPlan` target so its mission resumes,
    GUIDED for a guided-flown one) and re-arm. Monotone `seq`/`after` triggers
    never clear; `InterventionPlan(resume=True)` releases from inside the plan.

    Trigger timing (`after`, `dwell`) runs off the vehicle's own boot clock from
    telemetry, not wall time, so `speedup` and host load don't change *when* an
    intervention fires relative to the mission.
    """

    def __init__(
        self,
        sysid: int,
        intervention: Intervention,
        cmd_conn: MAVConnection,
        gra_origin: GRA,
        data_logger: DataLogger | None = None,
        source: str = "GCS",
    ) -> None:
        """
        Bind the intervention's plan to a `MAVLinkManager` on `cmd_conn` and
        start a position stream. `source` tags the log lines ("GCS" / "MITM");
        `data_logger` defaults to the GCS command log, so a MITM run can pass its
        own and keep the two logs separate.
        """
        self.sysid = sysid
        self.trigger = intervention.trigger
        self.plan = intervention.plan
        self.firmware: Firmware = intervention.firmware
        self.gra_origin = gra_origin
        self._source = source
        cmd_conn.target_system = sysid
        cmd_conn.target_component = 1
        self._mav = MAVLinkManager(
            conn=cmd_conn,
            data_logger=data_logger
            or DataLogger(path=DATA_PATH / "gcs_cmd", sysid=sysid),
        )
        self.plan.bind(self.gra_origin, self._mav)
        self._mav.send(
            ask_msg(cmd_conn, MsgID.GLOBAL_POSITION_INT, interval=_POSITION_INTERVAL_US)
        )
        self._current_seq: int | None = None
        self._total: int | None = None
        self._last_custom_mode: int | None = None
        # Vehicle boot clock (sim s) at the first tick with telemetry — the
        # anchor `after` counts from; `_seq_reached_at` likewise for `dwell`.
        self._veh_t0: float | None = None
        self._seq_reached_at: float | None = None
        self._engaged = False
        self._release: Action[Step] | None = None
        self._release_mode_name = ""

    def feed(self, msg: mavlink.MAVLink_message) -> None:
        """
        Cache one telemetry message and pull out what the trigger/release need.

        - `MISSION_CURRENT.seq` — index of the mission item the vehicle is
          flying now; drives `MissionTrigger` seq/final checks.
        - `MISSION_CURRENT.total` — sequence of the *last* mission item; older
          dialects omit it, so the last non-zero value is kept.
        - `HEARTBEAT.custom_mode` — the autopilot's flight-mode number
          (ArduCopter GUIDED=4, AUTO=3, …), restored on release. Read only from
          component id 1 (the flight controller); the GCS and Logic proxy also
          heartbeat on this link and their `custom_mode` is 0.

        `after` / `dwell` timing reads the vehicle's boot clock via
        `state.sim_time_s()` (see `_holds`), so it counts sim seconds.
        """
        self._mav.state.update(msg)
        msg_type = msg.get_type()
        if msg_type == "MISSION_CURRENT":
            mission_current = cast("mavlink.MAVLink_mission_current_message", msg)
            self._current_seq = int(mission_current.seq)
            total = getattr(mission_current, "total", 0)
            if total:
                self._total = int(total)
        elif msg_type == "HEARTBEAT" and msg.get_srcComponent() == 1:
            heartbeat = cast("mavlink.MAVLink_heartbeat_message", msg)
            self._last_custom_mode = int(heartbeat.custom_mode)

    def tick(self) -> None:
        """
        Advance the intervention one step, according to the trigger.

        - not engaged + trigger holds -> take over: snapshot the release mode,
          reset and start the plan;
        - engaged + trigger holds      -> run the next plan step;
        - engaged + trigger cleared    -> switch the vehicle back to the release
          mode; once that completes, re-arm for a possible next engage.
        """
        if self._holds():
            if not self._engaged:
                self._prepare_release()
                logging.info(
                    f"{self._source} intervention: taking over vehicle "
                    f"{self.sysid} (will restore {self._release_mode_name})"
                )
                self.plan.reset()
                self._engaged = True
            self.plan.act()
        elif self._engaged and self._release is not None:
            self._release.act()
            if self._release.state == State.DONE:
                logging.info(
                    f"{self._source} intervention: released vehicle "
                    f"{self.sysid} to {self._release_mode_name}"
                )
                self._release = None
                self.plan.reset()
                self._engaged = False

    def _prepare_release(self) -> None:
        """
        Snapshot the vehicle's current mode and build the action to restore it.

        Also patches the plan's own `resume=True` tail (if any) to the same
        mode via `set_resume_mode`, so a plan-driven release under a monotone
        trigger restores the real pre-takeover mode too, not just AUTO.
        """
        mode = self._release_mode()
        self._release_mode_name = mode.name
        self._release = make_set_mode(mode)
        self._release.bind(self.gra_origin, self._mav)
        set_resume_mode = getattr(self.plan, "set_resume_mode", None)
        if set_resume_mode is not None:
            set_resume_mode(mode)

    def _release_mode(self) -> CopterMode | PlaneMode:
        """
        Flight mode to restore on release: the vehicle's current mode if it is
        known and recognised, else the firmware's AUTO as a fallback.
        """
        mode_cls = CopterMode if self.firmware == "ArduCopter" else PlaneMode
        if self._last_custom_mode is not None:
            try:
                return mode_cls(self._last_custom_mode)
            except ValueError:
                logging.warning(
                    "%s intervention: vehicle %s in unrecognised mode %d; "
                    "will release to AUTO instead",
                    self._source,
                    self.sysid,
                    self._last_custom_mode,
                )
        return auto_mode(self.firmware)

    def _holds(self) -> bool:
        """Whether the trigger says the GCS should be in control this tick."""
        now = self._mav.state.sim_time_s()
        if now is None:
            # No vehicle clock yet. Also no position, so only a bare
            # `MissionTrigger(seq=)` could fire — let it, with elapsed 0.
            return self.trigger.holds(
                TriggerContext(current_seq=self._current_seq, total=self._total)
            )
        if self._veh_t0 is None:
            self._veh_t0 = now
        self._note_seq_milestone(now)
        seq_elapsed = (
            None if self._seq_reached_at is None else now - self._seq_reached_at
        )
        return self.trigger.holds(
            TriggerContext(
                current_seq=self._current_seq,
                elapsed=now - (self._veh_t0 or now),
                seq_elapsed=seq_elapsed,
                position=self._position(),
                engaged=self._engaged,
                total=self._total,
            )
        )

    def _note_seq_milestone(self, now: float) -> None:
        """
        Record (once) when the trigger's `seq` / `final` point is first reached.

        `_holds` turns that into `seq_elapsed`, which `MissionTrigger.dwell`
        measures from. `ProximityTrigger` has no `seq` / `final`, hence the
        `getattr` defaults — for it this is a no-op.
        """
        if self._seq_reached_at is not None:
            return
        seq = getattr(self.trigger, "seq", None)
        at_seq = (
            seq is not None
            and self._current_seq is not None
            and self._current_seq >= seq
        )
        on_final = (
            getattr(self.trigger, "final", False)
            and self._current_seq is not None
            and self._total is not None
            and self._total >= 1
            and self._current_seq >= self._total
        )
        if at_seq or on_final:
            self._seq_reached_at = now

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
