"""Dataclass config for Oracle-tunable parameters."""

from dataclasses import dataclass


@dataclass(kw_only=True)
class OracleConfig:
    """Tunable Oracle parameters, decoupled from Simulator-provided wiring."""

    transmission_range: float = 100.0  # meters, inter-Vehicle communication range
