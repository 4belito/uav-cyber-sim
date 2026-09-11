"""Init file for enums package."""

from __future__ import annotations

from .airspeed import AirSpeed
from .brdtype import BRDType
from .frameclass import FrameClass
from .frametype import FrameType
from .motpwmtype import MOTPWMType
from .wpnav import WPNav

__all__ = [
    "AirSpeed",
    "BRDType",
    "FrameClass",
    "FrameType",
    "MOTPWMType",
    "WPNav",
]
