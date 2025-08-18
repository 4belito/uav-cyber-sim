"""
Gazebo Simulator Module.

This module defines a Gazebo-based simulator that extends the base Simulator class.
It dynamically generates UAV model files, launches ArduPilot and logic processes,
and modifies Gazebo world files to include drones and waypoint markers.

Main Features:
- Supports custom models and color-coded UAVs
- Dynamically generates `model.sdf` files for each UAV
- Updates existing Gazebo world files to include UAVs and waypoint markers
- Launches Gazebo with the customized world file

"""

import logging
import os
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from config import ARDUPILOT_GAZEBO_MODELS, ENV_CMD_GAZ
from helpers import create_process
from helpers.change_coordinates import heading_to_yaw
from mavlink.customtypes.location import XYZRPY, GRAPose
from simulator.gazebo.config import COLOR_MAP, ConfigGazebo, GazVehicle, GazWP
from simulator.visualizer import Visualizer


class Gazebo(Visualizer[GazVehicle]):
    """
    Gazebo-specific simulator that launches UAVs in a Gazebo world.
    It configures drone models, world markers, and coordinates with ArduPilot logic.
    """

    name = "Gazebo"
    delay = False

    def __init__(
        self,
        config: ConfigGazebo,
        gra_origin: GRAPose,
    ):
        super().__init__(config)
        self.origin_str = ",".join(map(str, gra_origin))
        self.config: ConfigGazebo = config

    def add_vehicle_cmd(self, i: int) -> str:
        """Add gazebo model (only iris TODO: add others)."""
        return f" -f gazebo-iris --custom-location={self.origin_str}"

    def launch(self, port_offsets: list[int]):
        """Launch the Gazebo simulator with the specified UAV and waypoints."""
        base_models = [f"{veh.model}_{veh.color}" for veh in self.config.vehicles]
        self._generate_drone_models_from_bases(base_models, base_port_in=9002, step=10)
        updated_world = self._update_world(self.config.world_path)
        create_process(f"gazebo {updated_world}", visible=False, env_cmd=ENV_CMD_GAZ)
        logging.info("🖥️ Gazebo launched for realistic simulation and 3D visualization.")

    def _generate_drone_models_from_bases(
        self,
        base_models: list[str],
        base_port_in: int = 9002,
        step: int = 10,
    ) -> None:
        template_path = Path(ARDUPILOT_GAZEBO_MODELS) / "drone"
        output_dir = Path(ARDUPILOT_GAZEBO_MODELS)
        output_dir.mkdir(parents=True, exist_ok=True)

        for i in range(self.config.n_vehicles):
            name = f"drone{i + 1}"
            new_model_path = output_dir / name
            if new_model_path.exists():
                shutil.rmtree(new_model_path)
            shutil.copytree(template_path, new_model_path)

            sdf_path = new_model_path / "model.sdf"
            with open(sdf_path, "r", encoding="utf-8") as f:
                sdf = f.read()

            sdf = re.sub(r'<model name="[^"]+">', f'<model name="{name}">', sdf)
            sdf = re.sub(
                r"<include>\s*<uri>model://[^<]+</uri>\s*</include>",
                f"<include>\n  <uri>model://{base_models[i]}</uri>\n</include>",
                sdf,
            )

            port_in = base_port_in + i * step
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

    def _update_world(self, world_path: str) -> str:
        updated_world_path = os.path.expanduser(world_path[:-6] + "_updated.world")
        tree = ET.parse(world_path)
        root = tree.getroot()
        world_elem = root.find("world")

        if world_elem is None:
            raise ValueError("Could not find 'world' element in the XML.")

        self._remove_old_models(world_elem)
        self._add_marker_elements(world_elem)
        self._add_drone_elements(world_elem)

        tree.write(updated_world_path)
        return updated_world_path

    def _remove_old_models(self, world_elem: ET.Element) -> None:
        for model in world_elem.findall("model"):
            model_name = model.attrib.get("name", "")
            if model_name in {"green_waypoint", "red_waypoint", "drone", "iris_demo"}:
                world_elem.remove(model)

    def _add_marker_elements(self, world_elem: ET.Element) -> None:
        for i, veh in enumerate(self.config.vehicles):
            for j, waypoint in enumerate(veh.mtraj):
                marker_elem = self._generate_waypoint_element(waypoint, i, j)
                world_elem.append(marker_elem)

    def _add_drone_elements(self, world_elem: ET.Element) -> None:
        for i, veh in enumerate(self.config.vehicles):
            x, y, z, h = veh.home
            pose = XYZRPY(x, y, z, 0, 0, heading_to_yaw(h))
            drone_elem = self._generate_drone_element(f"drone{i + 1}", pose)
            world_elem.append(drone_elem)

    def _generate_waypoint_element(
        self, w: GazWP, traj_id: int, way_id: int
    ) -> ET.Element:
        model = ET.Element("model", name=f"waypoint_{traj_id}.{way_id}")
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

    def _add_visual(self, visual: ET.Element, w: GazWP) -> None:
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

    def _generate_drone_element(self, instance_name: str, pose: XYZRPY) -> ET.Element:
        model = ET.Element("model", name=instance_name)
        ET.SubElement(model, "pose").text = f"{pose}"
        include = ET.SubElement(model, "include")
        ET.SubElement(include, "uri").text = f"model://{instance_name}"
        return model
