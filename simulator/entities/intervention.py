"""GCS intervention: a trigger plus a guided plan the GCS applies to a vehicle."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Protocol

from simulator.helpers.coordinates import ENU

if TYPE_CHECKING:
    from simulator.config import Firmware
    from simulator.planner.plan import Plan

TriggerMode = Literal["any", "all"]


@dataclass
class TriggerContext:
    """
    What a trigger gets to look at, sampled once per monitor tick.

    `elapsed` is seconds since the GCS started monitoring the vehicle;
    `seq_elapsed` is seconds since the mission reached the trigger's `seq` (None
    until then); `engaged` says whether the GCS already holds control, which is
    what lets a guard apply hysteresis. `total` is `MISSION_CURRENT.total` — the
    sequence of the last mission item — used by `MissionTrigger(final=True)`.
    """

    current_seq: int | None = None
    elapsed: float = 0.0
    seq_elapsed: float | None = None
    position: ENU | None = None
    engaged: bool = False
    total: int | None = None


class Trigger(Protocol):
    """
    Decides whether the GCS should be holding control right now.

    Re-evaluated every monitor tick: the runner engages while it holds and
    releases when it stops holding. Implementations differ in **lifecycle** —
    `MissionTrigger` is an event (monotone, so control is taken and kept), while
    `ProximityTrigger` is a state (reversible, so control is handed back).
    """

    def holds(self, ctx: TriggerContext) -> bool:
        """Whether the GCS should hold control at this tick."""
        ...

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON, tagged with its kind."""
        ...


@dataclass
class MissionTrigger:
    """
    Take control at a point in the mission, or after a delay.

    * `seq` — a mission-sequence point: satisfied once the vehicle's
      `MISSION_CURRENT.seq` reaches it. `dwell` optionally delays it by that many
      seconds *after* the seq point is reached (so `seq=3, dwell=10` = "10 s after
      waypoint 3").
    * `after` — a delay in seconds measured from **when the GCS starts monitoring
      the vehicle** (the beginning), not from the seq point.
    * `final` — satisfied once the vehicle reaches its **last** mission item,
      whatever its sequence number. For an `AutoPlan` that item is the `NAV_LAND`,
      so `final=True` fires "as the drone starts landing" without the caller
      counting waypoints (the LAND seq shifts every time a waypoint is
      added/removed). Resolved from `MISSION_CURRENT.total`, which ArduPilot sets
      to the last item's sequence. `dwell` delays it too — `final=True, dwell=5`
      fires 5 s after the last item goes active.
    * `descending_below` — satisfied once the vehicle is on its last mission item
      **and** its altitude (ENU up, metres above the run origin) has dropped
      below this value: i.e. the landing descent is actually underway. Unlike a
      bare altitude check it does not fire during the takeoff climb, and unlike
      `dwell` it tracks the vehicle rather than the wall clock, so it is
      unaffected by `speedup`. Needs `GLOBAL_POSITION_INT` (a blackout MITM that
      drops it keeps this from firing).

    `dwell` requires `seq` or `final`; it measures from whichever is reached
    first. At least one of `seq`, `after`, `final`, `descending_below` must be
    set; `mode` combines them when several are (`"any"` on whichever hits first,
    `"all"` requires all).

    These conditions are **monotone** — once satisfied they stay satisfied — so an
    intervention using this trigger takes control and never gives it back. Use
    `ProximityTrigger` for a condition that can clear again.
    """

    seq: int | None = None
    after: float | None = None
    dwell: float | None = None
    final: bool = False
    descending_below: float | None = None
    mode: TriggerMode = "any"
    # Latches once any condition is first met, so the trigger stays satisfied
    # even for a reversible condition like `descending_below` (the GCS climbing
    # the vehicle back up must not un-fire the trigger and hand it to AUTO).
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
        """Serialize for the GCS-process config JSON, tagged with its kind."""
        return {
            "kind": "mission",
            "seq": self.seq,
            "after": self.after,
            "dwell": self.dwell,
            "final": self.final,
            "descending_below": self.descending_below,
            "mode": self.mode,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> MissionTrigger:
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
class ProximityTrigger:
    """
    Hold control while the vehicle is near a point — a guard, not an event.

    Distance is **horizontal** (East/North only), so a tall hazard such as a tower
    guards at every altitude and the point's `z` does not matter.

    Unlike `MissionTrigger` this is **reversible**, which is what lets the GCS give
    the vehicle back: it engages inside `radius`, keeps control until the vehicle
    is clear past `release_radius` (hysteresis, so it does not chatter at the
    boundary; defaults to `radius`), and engages again if the vehicle returns.
    """

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
        # Wider threshold once engaged, so the vehicle must clearly leave the zone
        # before the GCS lets go.
        limit = self.radius
        if ctx.engaged and self.release_radius is not None:
            limit = self.release_radius
        return (
            math.hypot(ctx.position.x - self.near.x, ctx.position.y - self.near.y)
            < limit
        )

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON, tagged with its kind."""
        return {
            "kind": "proximity",
            "near": self.near._asdict(),
            "radius": self.radius,
            "release_radius": self.release_radius,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> ProximityTrigger:
        """Rebuild a ProximityTrigger from its serialized form."""
        release_radius = data.get("release_radius")
        return cls(
            near=ENU(**data["near"]),
            radius=float(data["radius"]),
            release_radius=(None if release_radius is None else float(release_radius)),
        )


_TRIGGER_KINDS: dict[str, type[MissionTrigger] | type[ProximityTrigger]] = {
    "mission": MissionTrigger,
    "proximity": ProximityTrigger,
}


def build_trigger(data: Mapping[str, Any]) -> Trigger:
    """Rebuild the trigger described by a serialized `{"kind": ...}` mapping."""
    kind = data.get("kind", "mission")
    trigger_cls = _TRIGGER_KINDS.get(kind)
    if trigger_cls is None:
        raise ValueError(
            f"Unknown trigger kind {kind!r}; expected one of {sorted(_TRIGGER_KINDS)}"
        )
    return trigger_cls.from_dict(data)


@dataclass
class Intervention:
    """
    A GCS intervention: a `Trigger` and the guided `Plan` to run while it holds.

    The plan is any registered `Plan` exposing a spec (typically an
    `InterventionPlan`). It crosses to the GCS process as
    `{trigger, plan_spec, firmware}` and is rebuilt there with `Plan.build`.

    `firmware` is what the GCS commands to hand control back: when the trigger
    stops holding it switches the vehicle to that firmware's AUTO mode, and the
    paused mission resumes from where it left off. With a `MissionTrigger` that
    never happens, since those conditions never clear.
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
        """Rebuild an Intervention (trigger + plan) from its serialized form."""
        # Imported here, not at module load, to keep this entity free of the
        # planner import chain (which loops back through entities).
        from simulator.planner.plan import Plan, PlanSpec

        return cls(
            trigger=build_trigger(data["trigger"]),
            plan=Plan.build(PlanSpec(**data["plan_spec"])),
            firmware=data.get("firmware", "ArduCopter"),
        )
