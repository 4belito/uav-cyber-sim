"""Protocol for vehicle information."""

from typing import Protocol, TypedDict


class FrameOptions(TypedDict):
    waf_target: str
    default_params_filename: str | list[str]
    model: str


class VehicleInfoProtocol(Protocol):
    """
    Protocol defining the expected interface for vehicle
    information used in the SITL simulator.
    """

    def options_for_frame(
        self,
        frame: str,
        vehicle: str,
        opts: object,
    ) -> FrameOptions:
        """Return a dictionary of options for the given frame and vehicle type."""
        ...
