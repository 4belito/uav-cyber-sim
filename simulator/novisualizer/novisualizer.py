"""Minimal visualizer that sets home locations without GUI rendering."""

import logging
from dataclasses import dataclass

from helpers.coordinates import ENUPose, GRAPose
from simulator.visualizer import ConfigVis, Visualizer


@dataclass
class NovisVehicle:
    """Vehicle with a home position."""

    home: GRAPose


class ConfigNovis(ConfigVis[NovisVehicle]):
    """Stores the GRA origin used to compute UAV home positions."""

    def __init__(
        self,
        origin: GRAPose,
    ) -> None:
        super().__init__()
        self.origin = origin

    def add(
        self,
        base_home: ENUPose,
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home = self.origin.to_abs(base_home)
        self.add_vehicle(NovisVehicle(home=home))


class NoVisualizer(Visualizer[NovisVehicle]):
    """No-op visualizer for headless simulation."""

    name = "novis"

    def __init__(
        self,
        config: ConfigNovis,
    ):
        super().__init__()
        self.config = config

    def add_vehicle_cmd(self, i: int):
        """Add GRA location to the vhecle comand."""
        homes_str = ",".join(map(str, self.config.vehicles[i].home))
        return f" --custom-location={homes_str}"

    def launch(self, port_offsets: list[int], verbose: int = 1):
        """Print a message indicating that no visualizer will be launched."""
        logging.info("🙈 Running without visualization.")
