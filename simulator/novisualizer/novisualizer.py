"""Minimal visualizer that sets home locations without GUI rendering."""

import logging
from dataclasses import dataclass

from helpers.coordinates import ENUPose, GRAPose
from simulator.vehicle import SimVehicle, Vehicle
from simulator.visualizer import Visualizer  # ConfigVis,


@dataclass
class NovisVehicle(Vehicle):
    """Vehicle with a home position."""

    home: ENUPose


NovisVehicles = list[NovisVehicle]


class NoVisualizer(Visualizer[NovisVehicle]):
    """No-op visualizer for headless simulation."""

    name = "novis"

    def __init__(
        self,
        gra_origin: GRAPose,
    ):
        super().__init__(gra_origin)

    def get_vehicle(self, vehicle: SimVehicle) -> NovisVehicle:
        """Convert a Vehicle to a NovisVehicle with GRA home position."""
        return NovisVehicle(home=vehicle.home)

    def add_vehicle_cmd(self, i: int) -> str:
        """Add GRA location to the vehicle command."""
        homes_str = self.gra_origin.to_abs(self.vehicles[i].home).to_str()
        return f" --custom-location={homes_str}"

    def launch(self, port_offsets: list[int], verbose: int = 1):
        """Print a message indicating that no visualizer will be launched."""
        logging.info("🙈 Running without visualization.")

    def show(self):
        """Print the vehicles."""
        print(self.vehicles)
