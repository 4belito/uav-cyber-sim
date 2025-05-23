import numpy as np
import os
import xml.etree.ElementTree as ET
import subprocess
import shutil
import re
from pathlib import Path

from typing import List, Tuple

from plan import Plan
from simulators.sim import Simulator, SimName
from helpers.change_coordinates import heading_to_yaw


# Define color mappings
class Color:
    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    ORANGE = "orange"
    YELLOW = "yellow"


COLOR_MAP = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
}

from config import ARDUPILOT_GAZEBO_MODELS


class Gazebo(Simulator):
    def __init__(
        self,
        offsets: List[Tuple],
        plans: List[Plan],
        world_path: str,
        models: List[str],
        colors: List[str],
        markers: np.ndarray,
    ):
        super().__init__(name=SimName.QGROUND, offsets=offsets, plans=plans)
        self.add_info("models", models)
        self.add_info("colors", colors)
        self.add_info("markers", markers)
        self.add_info("world_path", self.update_world(world_path))

    def _add_vehicle_cmd_fn(self, i):
        return " -f gazebo-iris"

    def _launch_application(self):
        base_models = [
            f"{self.info['models'][i]}_{self.info['colors'][i]}"
            for i in range(self.n_uavs)
        ]
        self.generate_drone_models_from_bases(
            base_models,  # ← now the only input that determines how many drones
            base_port_in=9002,
            step=10,
        )
        sim_cmd = ["gazebo", "--verbose", self.info["world_path"]]  #
        subprocess.Popen(
            sim_cmd,
            stdout=subprocess.DEVNULL,  # Suppress standard output
            stderr=subprocess.DEVNULL,  # Suppress error output
            shell=False,  # Ensure safety when passing arguments
        )

    def generate_drone_models_from_bases(
        self,
        base_models,  # ← now the only input that determines how many drones
        base_port_in=9002,
        step=10,
    ):
        template_path = Path(ARDUPILOT_GAZEBO_MODELS) / "drone"
        output_dir = Path(ARDUPILOT_GAZEBO_MODELS)
        output_dir.mkdir(parents=True, exist_ok=True)

        for i in range(self.n_uavs):
            name = f"drone{i+1}"
            new_model_path = output_dir / name

            # Overwrite existing folder
            if new_model_path.exists():
                shutil.rmtree(new_model_path)
            shutil.copytree(template_path, new_model_path)

            sdf_path = new_model_path / "model.sdf"
            with open(sdf_path, "r") as f:
                sdf = f.read()

            # Replace <model name="...">
            sdf = re.sub(r'<model name="[^"]+">', f'<model name="{name}">', sdf)

            # Replace <include><uri>...</uri></include>
            sdf = re.sub(
                r"<include>\s*<uri>model://[^<]+</uri>\s*</include>",
                f"<include>\n  <uri>model://{base_models[i]}</uri>\n</include>",
                sdf,
            )

            # Replace ports (if present)
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

            with open(sdf_path, "w") as f:
                f.write(sdf)

            # print(f"✅ Created {name} | Base: {base_model} | Ports: {port_in}-{port_out}")

    def generate_drone_element(self, instance_name, x, y, z, roll, pitch, yaw):
        import xml.etree.ElementTree as ET

        model = ET.Element("model", name=instance_name)

        pose = ET.SubElement(model, "pose")
        pose.text = f"{x} {y} {z} {roll} {pitch} {yaw}"

        include = ET.SubElement(model, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = f"model://{instance_name}"

        return model

    def update_world(self, world_path):
        # Save the modified world file
        updated_world_path = world_path[:-6] + "_updated.world"
        updated_world_path = os.path.expanduser(updated_world_path)

        # Load the existing SDF file
        tree = ET.parse(world_path)
        root = tree.getroot()

        # Find the <world> element
        world_elem = root.find("world")

        # Remove old markers and drones
        for model in world_elem.findall("model"):
            model_name = model.attrib.get("name", "")
            if model_name in ["green_waypoint", "red_waypoint", "drone", "iris_demo"]:
                world_elem.remove(model)

        # Add makers
        for marker_name, marker_data in self.info["markers"].items():
            ## Add markers
            positions = marker_data.pop("pos")
            for j, (x, y, z) in enumerate(positions):
                marker_elem = self.generate_waypoint_element(
                    f"{marker_name}_{j}", x, y, z, **marker_data
                )
                world_elem.append(marker_elem)
                tree.write(updated_world_path)

        # Add vehicles
        for i in range(self.n_uavs):
            x, y, z, heading = self.offsets[i]
            # ensure there is enough model folders with model_name(this may be changedto directly write the code as is done for markers)
            drone_elem = self.generate_drone_element(
                f"drone{i+1}", x, y, z, 0, 0, heading_to_yaw(heading)
            )
            world_elem.append(drone_elem)
            tree.write(updated_world_path)

        # Save the modified world file
        updated_world_path = world_path[:-6] + "_updated.world"
        updated_world_path = os.path.expanduser(updated_world_path)
        tree.write(updated_world_path)

        return updated_world_path

    def generate_waypoint_element(
        self, name, x, y, z, color="green", radius=0.2, alpha=0.05
    ):
        """Creates a fully defined XML element for a waypoint model with configurable color, radius, and transparency."""

        diffuse_color = COLOR_MAP.get(color.lower(), COLOR_MAP["green"])

        model = ET.Element("model", name=name)

        pose = ET.SubElement(model, "pose")
        pose.text = f"{x} {y} {z} 0 0 0"

        link = ET.SubElement(model, "link", name="link")

        # Inertial properties
        inertial = ET.SubElement(link, "inertial")
        for tag, value in {
            "mass": "1",
            "ixx": "0.1",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.1",
            "iyz": "0",
            "izz": "0.1",
        }.items():
            if tag == "mass":
                ET.SubElement(inertial, tag).text = value
            else:
                ET.SubElement(ET.SubElement(inertial, "inertia"), tag).text = value
        ET.SubElement(inertial, "pose").text = "0 0 0 0 -0 0"

        # Physics
        for tag in ["self_collide", "enable_wind", "kinematic", "gravity"]:
            ET.SubElement(link, tag).text = "0"
        ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"

        # Visual
        visual = ET.SubElement(link, "visual", name="visual")
        geometry = ET.SubElement(visual, "geometry")
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = str(radius)

        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        ET.SubElement(script, "name").text = "Gazebo/Grey"
        ET.SubElement(script, "uri").text = (
            "file://media/materials/scripts/gazebo.material"
        )

        shader = ET.SubElement(material, "shader", type="pixel")
        ET.SubElement(shader, "normal_map").text = "__default__"

        ET.SubElement(material, "ambient").text = "0.3 0.3 0.3 1"
        ET.SubElement(material, "diffuse").text = diffuse_color
        ET.SubElement(material, "specular").text = "0.01 0.01 0.01 1"
        ET.SubElement(material, "emissive").text = "0 0 0 1"

        ET.SubElement(visual, "pose").text = "0 0 0 0 -0 0"
        ET.SubElement(visual, "transparency").text = str(alpha)
        ET.SubElement(visual, "cast_shadows").text = "1"

        # Static properties
        ET.SubElement(model, "static").text = "0"
        ET.SubElement(model, "allow_auto_disable").text = "1"

        return model
