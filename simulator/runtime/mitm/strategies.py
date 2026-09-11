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
directly (e.g. ``InterventionStrategy(Intervention(...))``) and assigns it to
``vehicle.mitm``. This mirrors ``Plan``/``PlanSpec``
(``simulator/planner/plan.py``): each strategy builds its own ``self._spec``
so :meth:`MITMStrategy.build` can reconstruct it from JSON on the far side of
the ``simulator.mitm`` subprocess boundary.

:class:`InterventionStrategy` is the "hijack" attack: it runs a real GCS
:class:`~simulator.entities.intervention.Intervention` (trigger + guided plan)
from the man-in-the-middle position, reusing the exact same
:class:`~simulator.runtime.gcs_intervention.InterventionRunner` the GCS drives —
only the injection point differs.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any, ClassVar, TypeAlias, TypeVar, cast

from pymavlink.dialects.v20 import ardupilotmega as mavlink

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping, Sequence

    from simulator.entities.intervention import Intervention
    from simulator.helpers.connections import MAVConnection
    from simulator.helpers.coordinates import GRA
    from simulator.runtime.gcs_intervention import InterventionRunner

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

    `gra_origin` is the run's geodetic origin, threaded in so a strategy that
    reuses the planner machinery (`InterventionStrategy`) can resolve its ENU
    waypoints exactly as the GCS does.
    """

    def __init__(
        self,
        sysid: int,
        to_logic: MAVConnection,
        to_gcs: Sequence[MAVConnection],
        gra_origin: GRA,
    ) -> None:
        self.sysid = sysid
        self.gra_origin = gra_origin
        self._to_logic = to_logic
        self._to_gcs = to_gcs
        # srcSystem 255 -> injected commands look GCS-origin; srcSystem <sysid>
        # -> injected telemetry looks vehicle-origin.
        self._logic_encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=200)
        self._gcs_encoder = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=200)
        self._lock = threading.Lock()

    @property
    def command_conn(self) -> MAVConnection:
        """
        The Logic-facing command connection (spoofed as the GCS, srcSystem 255).

        A strategy that drives a full guided plan (`InterventionStrategy`) hands
        this to an `InterventionRunner` so its `MAVLinkManager` sends land at
        Logic just as the GCS's command channel would.
        """
        return self._to_logic

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

    def close(self) -> None:
        """Release any resources the strategy started (default: nothing)."""

    def get_spec(self) -> MITMSpec:
        """Get the specification of this strategy."""
        if self._spec is None:
            raise RuntimeError(f"{type(self).__name__} does not expose a specification")
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


@MITMStrategy.register("intervention")
class InterventionStrategy(MITMStrategy):
    """
    Run a GCS `Intervention` from the man-in-the-middle position (the hijack).

    Takes the very same object a GCS would — a `Trigger` plus a guided
    `InterventionPlan` — and drives it with the very same
    `InterventionRunner`; only the injection point differs. Relayed downlink
    telemetry is fed to the runner and it is ticked on every message, so the
    plan's waypoint margins and the trigger's `dwell` / `final` / proximity
    conditions all behave exactly as they do on the GCS side. Commands the
    runner emits are spoofed as the GCS (srcSystem 255) via the MITM's
    Logic-facing command connection.

    With a `MissionTrigger` this is the classic visible hijack: take control at
    a mission point and keep it, telemetry still flowing to the real GCS. With a
    `ProximityTrigger` the attacker hands control back when the condition
    clears, just like the GCS runner.

    Same target assumptions as a GCS intervention (`Intervention`): only a
    seq-based `MissionTrigger` needs the victim on an `AutoPlan`. Nothing
    enforces it.
    """

    def __init__(self, intervention: Intervention | Mapping[str, Any]) -> None:
        super().__init__()
        # Local import: keeps the planner chain out of every MITM process.
        from simulator.entities.intervention import Intervention as _Intervention

        iv_dict = (
            intervention.to_dict()
            if isinstance(intervention, _Intervention)
            else dict(intervention)
        )
        # Rebuild now so a malformed intervention fails fast in the notebook.
        self._intervention = _Intervention.from_dict(iv_dict)
        self._runner: InterventionRunner | None = None
        self._spec = MITMSpec(
            strategy_class="intervention",
            kwargs={"intervention": iv_dict},
        )

    def bind(self, ctx: MITMContext) -> None:
        """Build the `InterventionRunner` against the Logic-facing command link."""
        super().bind(ctx)
        from simulator.config import DATA_PATH
        from simulator.helpers.logging.data_logger import DataLogger
        from simulator.runtime.gcs_intervention import InterventionRunner

        self._runner = InterventionRunner(
            sysid=ctx.sysid,
            intervention=self._intervention,
            cmd_conn=ctx.command_conn,
            gra_origin=ctx.gra_origin,
            data_logger=DataLogger(path=DATA_PATH / "mitm_cmd", sysid=ctx.sysid),
            source="MITM",
        )

    def on_downlink(self, msg: MAVMsg) -> MAVMsg | None:
        """Feed telemetry to the intervention runner and advance it one tick."""
        runner = self._runner
        if runner is not None:
            runner.feed(msg)
            runner.tick()
        return msg  # visible hijack: telemetry still flows to the GCS


#: Generous bound on distinguishable GCS-per-vehicle count for `target_mask`.
_SPOOF_MAX_GCS = 32


@MITMStrategy.register("spoof_gcs")
class SpoofGCSStrategy(MITMStrategy):
    """
    Inject a fabricated GLOBAL_POSITION_INT toward a chosen subset of GCS.

    Watches ``MISSION_CURRENT`` on the relayed telemetry; once
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
            mission_current = cast("mavlink.MAVLink_mission_current_message", msg)
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
