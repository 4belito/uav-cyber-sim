"""Visualization vehicle."""

from dataclasses import dataclass
from typing import TypeVar


@dataclass
class Vehicle:
    """Base vehicle class."""

    model: str


VehT = TypeVar("VehT", bound=Vehicle)
