"""Simulator GCS configuration entity."""

from dataclasses import dataclass, field

from simulator.helpers.processes import SimProcess


@dataclass
class SimGCS:
    """
    Simulator GCS configuration.

    `verbose`/`terminals`/`suppress` override the Simulator-wide defaults for
    this GCS only; leave them `None` to inherit the Simulator's settings.
    """

    name: str
    sysids: list[int] = field(default_factory=lambda: [])
    verbose: int | None = None
    terminals: list[SimProcess] | None = None
    suppress: list[SimProcess] | None = None
