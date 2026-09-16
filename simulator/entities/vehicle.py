"""Visualization vehicle."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, TypeVar

if TYPE_CHECKING:
    from simulator.config import Model


@dataclass
class Vehicle:
    """Base vehicle class."""

    model: Model


VehT = TypeVar("VehT", bound=Vehicle)
