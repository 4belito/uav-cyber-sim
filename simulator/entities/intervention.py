"""GCS intervention: a trigger plus a guided plan the GCS applies to a vehicle."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, ClassVar, Literal, Self

from simulator.helpers.coordinates import ENU

if TYPE_CHECKING:
    from collections.abc import Mapping

    from simulator.config import Firmware
    from simulator.planner.plan import Plan

TriggerMode = Literal["any", "all"]


@dataclass
class TriggerContext:
    """
    The per-tick snapshot a trigger reads, every field filled from telemetry.

    - `current_seq` — current `MISSION_CURRENT.seq` (`None` before any item).
    - `elapsed` — **sim** seconds (vehicle boot clock) since monitoring started,
      so `speedup` does not shift trigger timing.
    - `seq_elapsed` — sim seconds since the mission reached the trigger's `seq`
      (`None` until then).
    - `position` — vehicle ENU relative to the run origin (`None` if unknown).
    - `engaged` — whether the GCS already holds control; lets a guard widen its
      release threshold so control doesn't chatter at the boundary.
    - `total` — `MISSION_CURRENT.total`: the *last* item's sequence number
      (`current_seq == total` on it), not a count. Drives `final` /
      `descending_below`.
    """

    current_seq: int | None = None
    elapsed: float = 0.0
    seq_elapsed: float | None = None
    position: ENU | None = None
    engaged: bool = False
    total: int | None = None


class Trigger(ABC):
    """
    Decides whether the GCS should hold control, re-evaluated every monitor tick.

    Lifecycle differs by subclass: `MissionTrigger` is monotone (take and keep),
    `ProximityTrigger` is reversible (hand back). Each sets a `KIND` and is
    auto-registered by it, so `Trigger.build` rebuilds it from its `to_dict`
    mapping across the process boundary — as `Plan` / `MITMStrategy` do.
    """

    KIND: ClassVar[str]
    _REGISTRY: ClassVar[dict[str, type[Trigger]]] = {}

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        kind = getattr(cls, "KIND", None)
        if kind is not None:
            Trigger._REGISTRY[kind] = cls

    @abstractmethod
    def holds(self, ctx: TriggerContext) -> bool:
        """Whether the GCS should hold control at this tick."""

    @abstractmethod
    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON, tagged with `KIND`."""

    @classmethod
    @abstractmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Self:
        """Rebuild this trigger from the mapping `to_dict` produced."""

    @staticmethod
    def build(data: Mapping[str, Any]) -> Trigger:
        """Rebuild whichever trigger a serialized `{"kind": ...}` mapping names."""
        kind = data.get("kind", "mission")
        trigger_cls = Trigger._REGISTRY.get(kind)
        if trigger_cls is None:
            raise ValueError(
                f"Unknown trigger kind {kind!r}; "
                f"expected one of {sorted(Trigger._REGISTRY)}"
            )
        return trigger_cls.from_dict(data)


@dataclass
class MissionTrigger(Trigger):
    """
    Take control at a mission point or after a delay, and keep it.

    **Monotone**: latched by `_fired` once any condition is first met, so even a
    reversible-looking one like `descending_below` never hands control back (use
    `ProximityTrigger` for that).

    - `seq` — fires once `MISSION_CURRENT.seq` reaches it.
    - `after` — fires this many **sim** seconds after the GCS starts monitoring
      (vehicle clock, so `speedup` doesn't move it).
    - `final` — fires on the *last* mission item whatever its number (via
      `total`), so it survives waypoint renumbering; for an `AutoPlan` that item
      is the `NAV_LAND`.
    - `descending_below` — fires once on the last item *and* below this ENU-up
      altitude — the landing descent; needs `GLOBAL_POSITION_INT`.
    - `dwell` — extra sim-second delay from the `seq` / `final` point (requires
      one of them).
    - `mode` — `"any"` (first condition to hit) or `"all"` (every set one).

    At least one of `seq` / `after` / `final` / `descending_below` is required.
    """

    KIND: ClassVar[str] = "mission"

    seq: int | None = None
    after: float | None = None
    dwell: float | None = None
    final: bool = False
    descending_below: float | None = None
    mode: TriggerMode = "any"
    _fired: bool = field(default=False, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if (
            self.seq is None
            and self.after is None
            and not self.final
            and self.descending_below is None
        ):
            raise ValueError(
                "MissionTrigger needs seq, after, final=True, or descending_below"
            )
        if self.dwell is not None and self.seq is None and not self.final:
            raise ValueError(
                "MissionTrigger dwell requires a seq or final to delay from"
            )

    def _dwell_met(self, ctx: TriggerContext) -> bool:
        """Whether `dwell` has elapsed since the seq/final point was reached."""
        return self.dwell is None or (
            ctx.seq_elapsed is not None and ctx.seq_elapsed >= self.dwell
        )

    @staticmethod
    def _on_final_item(ctx: TriggerContext) -> bool:
        """Whether the vehicle is on its last mission item (via MISSION_CURRENT)."""
        return (
            ctx.current_seq is not None
            and ctx.total is not None
            and ctx.total >= 1
            and ctx.current_seq >= ctx.total
        )

    def holds(self, ctx: TriggerContext) -> bool:
        """Whether the GCS should hold control at this tick."""
        if self._fired:
            return True
        conditions: list[bool] = []
        if self.seq is not None:
            seq_met = ctx.current_seq is not None and ctx.current_seq >= self.seq
            conditions.append(seq_met and self._dwell_met(ctx))
        if self.final:
            conditions.append(self._on_final_item(ctx) and self._dwell_met(ctx))
        if self.descending_below is not None:
            conditions.append(
                self._on_final_item(ctx)
                and ctx.position is not None
                and ctx.position.z < self.descending_below
            )
        if self.after is not None:
            conditions.append(ctx.elapsed >= self.after)
        met = all(conditions) if self.mode == "all" else any(conditions)
        self._fired = met
        return met

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON, tagged with `KIND`."""
        return {
            "kind": self.KIND,
            "seq": self.seq,
            "after": self.after,
            "dwell": self.dwell,
            "final": self.final,
            "descending_below": self.descending_below,
            "mode": self.mode,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Self:
        """Rebuild a MissionTrigger from its serialized form."""
        after = data.get("after")
        dwell = data.get("dwell")
        descending_below = data.get("descending_below")
        return cls(
            seq=data.get("seq"),
            after=None if after is None else float(after),
            dwell=None if dwell is None else float(dwell),
            final=bool(data.get("final", False)),
            descending_below=(
                None if descending_below is None else float(descending_below)
            ),
            mode=data.get("mode", "any"),
        )


@dataclass
class ProximityTrigger(Trigger):
    """
    Hold control while the vehicle is within `radius` of `near`.

    Distance is **horizontal** only, so a tall hazard guards at every altitude.
    **Reversible**: once engaged it keeps control until the vehicle is past
    `release_radius` (hysteresis against boundary chatter; defaults to `radius`),
    then re-engages if the vehicle comes back.
    """

    KIND: ClassVar[str] = "proximity"

    near: ENU
    radius: float
    release_radius: float | None = None

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("ProximityTrigger radius must be positive")
        if self.release_radius is not None and self.release_radius < self.radius:
            raise ValueError("ProximityTrigger release_radius must be >= radius")

    def holds(self, ctx: TriggerContext) -> bool:
        """Whether the GCS should hold control at this tick."""
        if ctx.position is None:
            return False
        limit = (
            self.release_radius
            if ctx.engaged and self.release_radius is not None
            else self.radius
        )
        dist = math.hypot(ctx.position.x - self.near.x, ctx.position.y - self.near.y)
        return dist < limit

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON, tagged with `KIND`."""
        return {
            "kind": self.KIND,
            "near": self.near._asdict(),
            "radius": self.radius,
            "release_radius": self.release_radius,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Self:
        """Rebuild a ProximityTrigger from its serialized form."""
        release_radius = data.get("release_radius")
        return cls(
            near=ENU(**data["near"]),
            radius=float(data["radius"]),
            release_radius=(None if release_radius is None else float(release_radius)),
        )


@dataclass
class Intervention:
    """
    A `Trigger` plus the guided `Plan` the GCS runs while the trigger holds.

    `plan` must assume the target is **already airborne** (no arm/takeoff) — use
    `InterventionPlan`, not `GuidedPlan` / `AutoPlan`. A plan that *uploads* its
    own mission replaces the target's, so pair it only with a one-way
    `MissionTrigger`.

    Only `MissionTrigger`'s `seq` / `final` / `descending_below` need the target
    on an **`AutoPlan`** (they read `MISSION_CURRENT`). `MissionTrigger(after=...)`
    and `ProximityTrigger` are mode-agnostic, and on release the runner restores
    whatever flight mode the vehicle held when it engaged (AUTO for an
    `AutoPlan`, so its mission resumes; GUIDED for a guided-flown target) — so
    intervening on a GUIDED target works too. `firmware` only picks the AUTO
    fallback for that restore. Nothing is enforced.

    Serializes to `{trigger, plan_spec, firmware}`, rebuilt via `Trigger.build`
    + `Plan.build`.
    """

    trigger: Trigger
    plan: Plan
    firmware: Firmware = "ArduCopter"

    def to_dict(self) -> dict[str, Any]:
        """Serialize the intervention for the GCS-process config JSON."""
        return {
            "trigger": self.trigger.to_dict(),
            "plan_spec": self.plan.get_spec().to_dict(),
            "firmware": self.firmware,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Intervention:
        """
        Rebuild an Intervention (trigger + plan) from its serialized form.

        The planner import is deferred to keep this module out of the
        planner -> entities import cycle.
        """
        from simulator.planner.plan import Plan, PlanSpec

        return cls(
            trigger=Trigger.build(data["trigger"]),
            plan=Plan.build(PlanSpec(**data["plan_spec"])),
            firmware=data.get("firmware", "ArduCopter"),
        )
