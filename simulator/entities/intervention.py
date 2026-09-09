"""GCS intervention: a trigger plus a guided plan the GCS applies to a vehicle."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

if TYPE_CHECKING:
    from simulator.planner.plan import Plan

TriggerMode = Literal["any", "all"]


@dataclass
class Trigger:
    """
    When a GCS intervention fires.

    Two independent conditions, each against its own reference:

    * `seq` — a mission-sequence point: satisfied once the vehicle's
      `MISSION_CURRENT.seq` reaches it. `dwell` optionally delays it by that many
      seconds *after* the seq point is reached (so `seq=3, dwell=10` = "10 s after
      waypoint 3"); `dwell` requires `seq`.
    * `after` — a delay in seconds measured from **when the GCS starts monitoring
      the vehicle** (the beginning), not from the seq point.

    At least one of `seq`/`after` must be set. `mode` combines them when both are:
    `"any"` fires on whichever hits first, `"all"` requires both. A plain
    `Trigger(seq=3)` reproduces the legacy behavior.
    """

    seq: int | None = None
    after: float | None = None
    dwell: float | None = None
    mode: TriggerMode = "any"

    def __post_init__(self) -> None:
        if self.seq is None and self.after is None:
            raise ValueError("Trigger needs a seq, an after time, or both")
        if self.dwell is not None and self.seq is None:
            raise ValueError("Trigger dwell requires a seq to delay from")

    def ready(
        self,
        current_seq: int | None,
        elapsed: float,
        seq_elapsed: float | None = None,
    ) -> bool:
        """
        Whether the intervention should fire now.

        `elapsed` is seconds since monitoring began; `seq_elapsed` is seconds
        since the `seq` point was reached (None until then), used by `dwell`.
        """
        conditions: list[bool] = []
        if self.seq is not None:
            seq_met = current_seq is not None and current_seq >= self.seq
            if self.dwell is not None:
                seq_met = (
                    seq_met and seq_elapsed is not None and seq_elapsed >= self.dwell
                )
            conditions.append(seq_met)
        if self.after is not None:
            conditions.append(elapsed >= self.after)
        return all(conditions) if self.mode == "all" else any(conditions)

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the GCS-process config JSON."""
        return {
            "seq": self.seq,
            "after": self.after,
            "dwell": self.dwell,
            "mode": self.mode,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Trigger:
        """Rebuild a Trigger from its serialized form."""
        after = data.get("after")
        dwell = data.get("dwell")
        return cls(
            seq=data.get("seq"),
            after=None if after is None else float(after),
            dwell=None if dwell is None else float(dwell),
            mode=data.get("mode", "any"),
        )


@dataclass
class Intervention:
    """
    A GCS intervention: a `Trigger` and the guided `Plan` to run when it fires.

    The plan is any registered `Plan` exposing a spec (typically an
    `InterventionPlan`). It crosses to the GCS process as `{trigger, plan_spec}`
    and is rebuilt there with `Plan.build`.
    """

    trigger: Trigger
    plan: Plan

    def to_dict(self) -> dict[str, Any]:
        """Serialize the intervention for the GCS-process config JSON."""
        return {
            "trigger": self.trigger.to_dict(),
            "plan_spec": self.plan.get_spec().to_dict(),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Intervention:
        """Rebuild an Intervention (trigger + plan) from its serialized form."""
        # Imported here, not at module load, to keep this entity free of the
        # planner import chain (which loops back through entities).
        from simulator.planner.plan import Plan, PlanSpec

        return cls(
            trigger=Trigger.from_dict(data["trigger"]),
            plan=Plan.build(PlanSpec(**data["plan_spec"])),
        )
