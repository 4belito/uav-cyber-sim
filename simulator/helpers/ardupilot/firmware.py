"""Firmware-specific flight-mode helpers."""

from simulator.config import Firmware
from simulator.helpers.connections.mavlink.enums import CopterMode, PlaneMode


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
