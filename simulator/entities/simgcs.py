"""Simulator GCS configuration entity."""

from dataclasses import dataclass, field


@dataclass
class SimGCS:
    """Simulator GCS configuration."""

    name: str
    sysids: list[int] = field(default_factory=lambda: [])
