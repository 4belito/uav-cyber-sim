"""Protocol for vehicle information."""

from typing import Protocol


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
    ) -> dict[str, object]:
        """Return a dictionary of options for the given frame and vehicle type."""
        ...
