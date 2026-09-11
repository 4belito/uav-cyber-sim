"""Firmware-specific flight-mode helpers."""

from __future__ import annotations

from typing import TYPE_CHECKING

from simulator.helpers.connections.mavlink.enums import CopterMode, PlaneMode

if TYPE_CHECKING:
    from simulator.config import Firmware


def guided_mode(firmware: Firmware) -> CopterMode | PlaneMode:
    """Return the GUIDED mode enum for the given firmware."""
    match firmware:
        case "ArduCopter":
            return CopterMode.GUIDED
        case "ArduPlane":
            return PlaneMode.GUIDED


def auto_mode(firmware: Firmware) -> CopterMode | PlaneMode:
    """Return the AUTO (mission) mode enum for the given firmware."""
    match firmware:
        case "ArduCopter":
            return CopterMode.AUTO
        case "ArduPlane":
            return PlaneMode.AUTO


def reset_mode(firmware: Firmware) -> CopterMode | PlaneMode:
    """Return the post-mission reset mode for the given firmware."""
    match firmware:
        case "ArduCopter":
            return CopterMode.STABILIZE
        case "ArduPlane":
            return PlaneMode.MANUAL
