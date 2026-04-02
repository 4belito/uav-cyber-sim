"""Visualization vehicle."""

from dataclasses import dataclass
from typing import TypeVar


@dataclass
class Vehicle:
    """Base vehicle class."""


VehT = TypeVar("VehT", bound=Vehicle)
