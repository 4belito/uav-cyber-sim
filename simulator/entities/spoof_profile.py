"""
Remote ID spoof profile: what fake position to broadcast and when.

Kept free of any vehicle/runtime imports (only coordinates) so the RID manager
can import it without a cycle; `SimVehicle.spoof` carries one of these.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from simulator.helpers.coordinates import ENU

if TYPE_CHECKING:
    from collections.abc import Mapping


@dataclass
class SpoofProfile:
    """
    A timed schedule of fake Remote ID positions for a spoofing vehicle.

    `keyframes` are `(t_seconds, fake ENU)` pairs, where `t` is measured from when
    the vehicle's RID manager starts (roughly process start, before arm/takeoff).
    `position_at` resolves the position to broadcast at a given elapsed time:

    * `t < keyframes[0][0]`            → real position (spoofing not started yet),
    * between two keyframes            → linear interpolation of the ENU,
    * `t >= keyframes[-1][0]`          → held at the last keyframe's position,
    * `stop is not None and t > stop`  → real position again.

    A single keyframe is a constant fake position from its time onward — see the
    `constant` helper for that common case.
    """

    keyframes: list[tuple[float, ENU]]
    stop: float | None = None

    def __post_init__(self) -> None:
        if not self.keyframes:
            raise ValueError("SpoofProfile needs at least one keyframe")
        self.keyframes = sorted(self.keyframes, key=lambda kf: kf[0])

    @classmethod
    def constant(
        cls, fake_pos: ENU, *, start: float = 0.0, stop: float | None = None
    ) -> SpoofProfile:
        """Return a constant fake position, broadcast from `start` until `stop`."""
        return cls(keyframes=[(start, fake_pos)], stop=stop)

    def position_at(self, t: float) -> ENU | None:
        """Fake ENU to broadcast at time `t`; `None` means send the real one."""
        if self.stop is not None and t > self.stop:
            return None
        first_t, _ = self.keyframes[0]
        if t < first_t:
            return None
        last_t, last_pos = self.keyframes[-1]
        if t >= last_t:
            return last_pos
        # Find the bracketing keyframes and linearly interpolate between them.
        for (t0, p0), (t1, p1) in zip(self.keyframes, self.keyframes[1:]):
            if t1 > t0 and t0 <= t < t1:
                frac = (t - t0) / (t1 - t0)
                return ENU(
                    p0.x + (p1.x - p0.x) * frac,
                    p0.y + (p1.y - p0.y) * frac,
                    p0.z + (p1.z - p0.z) * frac,
                )
        return last_pos  # unreachable given the guards above

    def to_dict(self) -> dict[str, Any]:
        """Serialize for the logic-process config JSON."""
        return {
            "keyframes": [{"t": t, "pos": pos._asdict()} for t, pos in self.keyframes],
            "stop": self.stop,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> SpoofProfile:
        """Rebuild a profile from its serialized form (inverse of `to_dict`)."""
        keyframes = [(float(kf["t"]), ENU(**kf["pos"])) for kf in data["keyframes"]]
        stop = data.get("stop")
        return cls(keyframes=keyframes, stop=None if stop is None else float(stop))
