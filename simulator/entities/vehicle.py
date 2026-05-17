"""Visualization vehicle."""

from dataclasses import dataclass
from typing import TypeVar

from simulator.config import Model


@dataclass
class Vehicle:
    """Base vehicle class."""

    model: Model


VehT = TypeVar("VehT", bound=Vehicle)
