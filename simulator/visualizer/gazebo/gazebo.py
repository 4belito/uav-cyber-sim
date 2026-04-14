"""
Gazebo Simulator Module.

This module defines a Gazebo-based simulator that extends the base Simulator class.
It dynamically generates Vehicle model files, launches ArduPilot and logic processes,
and modifies Gazebo world files to include vehicles and waypoint markers.

Main Features:
- Supports custom models and color-coded Vehicles
- Dynamically generates `model.sdf` files for each Vehicle
- Updates existing Gazebo world files to include Vehicles and waypoint markers
- Launches Gazebo with the customized world file

"""

import logging
import os
import re
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from simulator.config import (
    ARDUPILOT_GAZEBO_MODELS,
    RUNTIME_GAZEBO_MODELS,
    RUNTIME_GAZEBO_WORLDS,
    Color,
)
from simulator.entities.simvehicle import SimVehicle, Vehicle
from simulator.helpers.coordinates import XYZRPY, ENUPose, GRAPose
from simulator.helpers.math import heading_to_yaw
from simulator.helpers.processes import create_process
from simulator.visualizer.gazebo.preview import GazMarker, GazMarkers, show_markers
from simulator.visualizer.visualizer import Visualizer

Trace = tuple[
    list[float], list[float], list[float], list[float], list[float], list[str]
]


COLOR_MAP: dict[Color, str] = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
    Color.WHITE: "1.0 1.0 1.0 1",
}


@dataclass
class GazVehicle(Vehicle):
    """Represents a vehicle with a model and a trajectory."""

    home: ENUPose
    model: str
    color: Color
    mtraj: GazMarkers


class Gazebo(Visualizer[GazVehicle]):
    """
    Gazebo-specific simulator that launches Vehicles in a Gazebo world.
    It configures vehicle models, world markers, and coordinates with ArduPilot logic.
    """

    def __init__(
        self,
        gra_origin: GRAPose,
        world_path: str,
    ):
        super().__init__(gra_origin)
        self.world_path = Path(world_path)
        self.markers: GazMarkers = []

    @property
    def name(self) -> str:
        """Name of the visualizer."""
        return "Gazebo"

    def home_str(self, vehicle: SimVehicle) -> str:
        """Return the home position of the vehicle as a string for Gazebo commands."""
        return self.gra_origin.to_str()

    def launch(self, port_offsets: dict[int, int]):
        """Launch the Gazebo simulator with the specified Vehicle and waypoints."""
        self._generate_vehicle_models_from_bases(
            base_port_in=9002, port_offsets=port_offsets
        )
        updated_world = self._update_world(self.world_path)

        create_process(
            f"gazebo {updated_world}",
            visible=False,
            suppress_output=True,
            env=self._build_gazebo_env(),
        )
        logging.info(
            "🖥️  Gazebo launched for realistic simulation and 3D visualization."
        )

    def show(
        self,
        title: str = "Trajectories",
        frames: tuple[float, float, float] = (0.2, 0.2, 0.2),
        ground: float | None = 0,
    ) -> None:
        """Render a 3D interactive plot of waypoint trajectories using Plotly."""
        show_markers(self.markers, title=title, frames=frames, ground=ground)

    def get_visvehicle(
        self,
        vehicle: SimVehicle,
        radius: float = 0.2,
        alpha: float = 0.05,
    ) -> GazVehicle:
        """Convert a Vehicle to a GazVehicle with markers for its trajectory."""
        markertraj: GazMarkers = []
        for i, pos in enumerate(vehicle.waypoints):
            gaz_marker = GazMarker(
                name=str(i),
                group=f"traj_{vehicle.sysid}",
                pos=pos,
                color=vehicle.color,
                radius=radius,
                alpha=alpha,
            )
            markertraj.append(gaz_marker)
            self.markers.append(gaz_marker)
        return GazVehicle(
            model=vehicle.model,
            color=vehicle.color,
            home=vehicle.home,
            mtraj=markertraj,
        )

    def _build_gazebo_env(self) -> dict[str, str]:
        runtime = str(RUNTIME_GAZEBO_MODELS)
        base = str(ARDUPILOT_GAZEBO_MODELS)

        env = {
            "GAZEBO_MODEL_PATH": f"{runtime}:{base}",
            "GAZEBO_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
            "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
            "LD_LIBRARY_PATH": "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
            "HOME": os.environ.get("HOME", ""),
        }

        if "DISPLAY" in os.environ:
            env["DISPLAY"] = os.environ["DISPLAY"]

        return env

    def _generate_vehicle_models_from_bases(
        self,
        port_offsets: dict[int, int],
        base_port_in: int = 9002,
    ) -> None:

        RUNTIME_GAZEBO_MODELS.mkdir(parents=True, exist_ok=True)

        for sysid, veh in self.vehicles.items():
            template_path = ARDUPILOT_GAZEBO_MODELS / veh.model / "template"
            name = f"vehicle_{sysid}"
            new_model_path = RUNTIME_GAZEBO_MODELS / name
            if new_model_path.exists():
                shutil.rmtree(new_model_path)
            shutil.copytree(template_path, new_model_path)

            sdf_path = new_model_path / "model.sdf"
            with open(sdf_path, encoding="utf-8") as f:
                sdf = f.read()

            sdf = re.sub(r'<model name="[^"]+">', f'<model name="{name}">', sdf)
            sdf = re.sub(
                r"<include>\s*<uri>model://[^<]+</uri>\s*</include>",
                f"<include>\n  <uri>model://{veh.model}/{veh.color.name.lower()}</uri>\n</include>",
                sdf,
            )

            port_in = base_port_in + port_offsets[sysid]
            port_out = port_in + 1
            sdf = re.sub(
                r"<fdm_port_in>\d+</fdm_port_in>",
                f"<fdm_port_in>{port_in}</fdm_port_in>",
                sdf,
            )
            sdf = re.sub(
                r"<fdm_port_out>\d+</fdm_port_out>",
                f"<fdm_port_out>{port_out}</fdm_port_out>",
                sdf,
            )

            with open(sdf_path, "w", encoding="utf-8") as f:
                f.write(sdf)

    def _update_world(self, world_path: Path) -> Path:
        RUNTIME_GAZEBO_WORLDS.mkdir(parents=True, exist_ok=True)
        out_path = RUNTIME_GAZEBO_WORLDS / world_path.name
        tree = ET.parse(world_path)
        root = tree.getroot()
        world_elem = root.find("world")

        if world_elem is None:
            raise ValueError("Could not find 'world' element in the XML.")

        self._add_markers_elements(world_elem)
        self._add_vehicle_elements(world_elem)

        tree.write(out_path)
        return out_path

    def _add_markers_elements(self, world_elem: ET.Element):
        for mark in self.markers:
            marker_elem = self._generate_waypoint_element(mark)
            world_elem.append(marker_elem)

    def _generate_waypoint_element(self, w: GazMarker) -> ET.Element:
        model = ET.Element("model", name=f"{w.group}.{w.name}")
        x, y, z = w.pos

        ET.SubElement(model, "pose").text = f"{x} {y} {z} 0 0 0"
        link = ET.SubElement(model, "link", name="link")

        self._add_inertial(link)
        self._add_link_flags(link)
        ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"

        visual = ET.SubElement(link, "visual", name="visual")
        self._add_visual(visual, w)

        ET.SubElement(model, "static").text = "0"
        ET.SubElement(model, "allow_auto_disable").text = "1"
        return model

    def _add_vehicle_elements(self, world_elem: ET.Element) -> None:
        for sysid, veh in self.vehicles.items():
            x, y, z, h = veh.home
            pose = XYZRPY(x, y, z, 0, 0, heading_to_yaw(h))
            vehicle_elem = self._generate_vehicle_element(f"vehicle_{sysid}", pose)
            world_elem.append(vehicle_elem)

    def _add_inertial(self, link: ET.Element) -> None:
        inertial = ET.SubElement(link, "inertial")
        inertia = ET.SubElement(inertial, "inertia")
        for tag, value in {
            "mass": "1",
            "ixx": "0.1",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.1",
            "iyz": "0",
            "izz": "0.1",
        }.items():
            target = inertial if tag == "mass" else inertia
            ET.SubElement(target, tag).text = value
        ET.SubElement(inertial, "pose").text = "0 0 0 0 -0 0"

    def _add_link_flags(self, link: ET.Element) -> None:
        for tag in ["self_collide", "enable_wind", "kinematic", "gravity"]:
            ET.SubElement(link, tag).text = "0"

    def _add_visual(self, visual: ET.Element, w: GazMarker) -> None:
        geometry = ET.SubElement(visual, "geometry")
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = str(w.radius)

        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        ET.SubElement(script, "name").text = "Gazebo/Grey"
        ET.SubElement(
            script, "uri"
        ).text = "file://media/materials/scripts/gazebo.material"

        shader = ET.SubElement(material, "shader", type="pixel")
        ET.SubElement(shader, "normal_map").text = "__default__"
        ET.SubElement(material, "ambient").text = "0.3 0.3 0.3 1"
        ET.SubElement(material, "diffuse").text = COLOR_MAP.get(w.color)
        ET.SubElement(material, "specular").text = "0.01 0.01 0.01 1"
        ET.SubElement(material, "emissive").text = "0 0 0 1"

        ET.SubElement(visual, "pose").text = "0 0 0 0 -0 0"
        ET.SubElement(visual, "transparency").text = str(w.alpha)
        ET.SubElement(visual, "cast_shadows").text = "1"

    def _generate_vehicle_element(self, instance_name: str, pose: XYZRPY) -> ET.Element:
        model = ET.Element("model", name=instance_name)
        ET.SubElement(model, "pose").text = f"{pose}"
        include = ET.SubElement(model, "include")
        ET.SubElement(include, "uri").text = f"model://{instance_name}"
        return model
