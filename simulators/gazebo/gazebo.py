"""
Gazebo Simulator Module

This module defines a Gazebo-based simulator that extends the base Simulator class.
It dynamically generates UAV model files, launches ArduPilot and logic processes,
and modifies Gazebo world files to include drones and waypoint markers.

Main Features:
- Supports custom models and color-coded UAVs
- Dynamically generates `model.sdf` files for each UAV
- Updates existing Gazebo world files to include UAVs and waypoint markers
- Launches Gazebo with the customized world file

"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET
import plotly.graph_objects as go


import numpy as np
from numpy.typing import NDArray

from config import ARDUPILOT_GAZEBO_MODELS, Color
from helpers.change_coordinates import Offset, heading_to_yaw
from plan import Plan
from simulators.sim import Simulator, VisualizerName

COLOR_MAP: Dict[Color, str] = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
}


@dataclass
class WaypointMarker:
    """
    Defines a visual waypoint marker with position, color, size, and transparency
    in Gazebo.
    """

    x: float
    y: float
    z: float
    color: Color = Color.GREEN
    radius: float = 0.2
    alpha: float = 0.05


TrajectoryMarker = List[WaypointMarker]
Model = Tuple[str, Color]


@dataclass
class ConfigGazebo:
    """
    Configuration object for initializing the Gazebo simulator.

    Attributes:
        world_path (str): Path to the base Gazebo world file.
        models (List[str]): List of base model names to use for each UAV.
        colors (List[str]): List of color names for each UAV.
        markers (np.ndarray): Dictionary-like structure with marker positions
        and properties.
    """

    world_path: str
    models: List[Model]
    markers: List[TrajectoryMarker]

    @staticmethod
    def create_trajectory_from_array(
        array: NDArray[np.float64],
        color: Color = Color.GREEN,
        radius: float = 0.2,
        alpha: float = 0.05,
    ) -> TrajectoryMarker:
        traj: List[WaypointMarker] = []
        for row in array:
            traj.append(
                WaypointMarker(
                    x=float(row[0]),
                    y=float(row[1]),
                    z=float(row[2]),
                    color=color,
                    radius=radius,
                    alpha=alpha,
                )
            )

        return traj


@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def __str__(self) -> str:
        return f"{self.x} {self.y} {self.z} {self.roll} {self.pitch} {self.yaw}"


class Gazebo(Simulator):
    """
    Gazebo-specific simulator that launches UAVs in a Gazebo world.
    It configures drone models, world markers, and coordinates with ArduPilot logic.
    """

    def __init__(self, offsets: List[Offset], plans: List[Plan], config: ConfigGazebo):
        super().__init__(name=VisualizerName.GAZEBO, offsets=offsets, plans=plans)
        self.config: ConfigGazebo = config

    def _add_vehicle_cmd_fn(self, i: int) -> str:
        return f" -f gazebo-{self.config.models[i][0]}"

    def _launch_visualizer(self) -> None:
        models = self.config.models
        base_models = [f"{models[i][0]}_{models[i][1]}" for i in range(self.n_uavs)]
        self._generate_drone_models_from_bases(base_models, base_port_in=9002, step=10)
        updated_world = self._update_world(self.config.world_path)
        sim_cmd = ["gazebo", updated_world]
        # pylint: disable=consider-using-with
        subprocess.Popen(
            sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=False
        )

    def _generate_drone_models_from_bases(
        self,
        base_models: List[str],
        base_port_in: int = 9002,
        step: int = 10,
    ) -> None:

        template_path = Path(ARDUPILOT_GAZEBO_MODELS) / "drone"
        output_dir = Path(ARDUPILOT_GAZEBO_MODELS)
        output_dir.mkdir(parents=True, exist_ok=True)

        for i in range(self.n_uavs):
            name = f"drone{i+1}"
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
        for i, traj in enumerate(self.config.markers):
            for j, waypoint in enumerate(traj):
                marker_elem = self._generate_waypoint_element(waypoint, i, j)
                world_elem.append(marker_elem)

    def _add_drone_elements(self, world_elem: ET.Element) -> None:
        for i, (x, y, z, heading) in enumerate(self.offsets):
            pose = Pose(x, y, z, 0, 0, heading_to_yaw(heading))
            drone_elem = self._generate_drone_element(f"drone{i + 1}", pose)
            world_elem.append(drone_elem)

    def _generate_waypoint_element(
        self, w: WaypointMarker, traj_id: int, way_id: int
    ) -> ET.Element:
        model = ET.Element("model", name=f"waypoint_{traj_id}.{way_id}")
        ET.SubElement(model, "pose").text = f"{w.x} {w.y} {w.z} 0 0 0"
        link = ET.SubElement(model, "link", name="link")

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

        for tag in ["self_collide", "enable_wind", "kinematic", "gravity"]:
            ET.SubElement(link, tag).text = "0"
        ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"

        visual = ET.SubElement(link, "visual", name="visual")
        geometry = ET.SubElement(visual, "geometry")
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = str(w.radius)

        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        ET.SubElement(script, "name").text = "Gazebo/Grey"
        ET.SubElement(script, "uri").text = (
            "file://media/materials/scripts/gazebo.material"
        )

        shader = ET.SubElement(material, "shader", type="pixel")
        ET.SubElement(shader, "normal_map").text = "__default__"
        ET.SubElement(material, "ambient").text = "0.3 0.3 0.3 1"
        ET.SubElement(material, "diffuse").text = COLOR_MAP.get(w.color)
        ET.SubElement(material, "specular").text = "0.01 0.01 0.01 1"
        ET.SubElement(material, "emissive").text = "0 0 0 1"

        ET.SubElement(visual, "pose").text = "0 0 0 0 -0 0"
        ET.SubElement(visual, "transparency").text = str(w.alpha)
        ET.SubElement(visual, "cast_shadows").text = "1"

        ET.SubElement(model, "static").text = "0"
        ET.SubElement(model, "allow_auto_disable").text = "1"

        return model

    def _generate_drone_element(self, instance_name: str, pose: Pose) -> ET.Element:
        model = ET.Element("model", name=instance_name)
        ET.SubElement(model, "pose").text = f"{pose}"
        include = ET.SubElement(model, "include")
        ET.SubElement(include, "uri").text = f"model://{instance_name}"
        return model

    @staticmethod
    def plot_3d_interactive(
        markers: List[TrajectoryMarker],
        title: str = "title",
        frames: Tuple[float, float, float] = (0.2, 0.2, 0.2),
        ground: float | None = 0,
    ):
        # Create a list to store all waypoints
        data: List[go.Scatter3d] = []
        all_x: List[float] = []
        all_y: List[float] = []
        all_z: List[float] = []
        for i, traj in enumerate(markers):
            xs: List[float] = []
            ys: List[float] = []
            zs: List[float] = []
            colors: List[Color] = []
            for w in traj:
                xs.append(w.x)
                ys.append(w.y)
                zs.append(w.z)
                colors.append(w.color)
            # Add a scatter plot for each marker set
            trace = go.Scatter3d(
                x=xs,
                y=ys,
                z=zs,
                mode="markers",
                marker=dict(size=6, color=colors),
                name=f"trajectory {i}",
            )
            data.append(trace)
            all_x += xs
            all_y += ys
            all_z += zs

        # Compute axis limits with scaling
        plot_limits = [
            (min(all_x), max(all_x)),
            (min(all_y), max(all_y)),
            (min(all_z), max(all_z)),
        ]
        ranges = [
            [m - frames[i] * (M - m), M + frames[i] * (M - m)]
            for i, (m, M) in enumerate(plot_limits)
        ]
        if ground is not None:
            ranges[2][0] = ground
        # Create figure with all markers
        fig = go.Figure(data=data)
        fig.update_layout(  # type: ignore
            title=dict(text=title, x=0.5, xanchor="center"),  # Centers the title
            scene=dict(
                xaxis_title="x",
                yaxis_title="y",
                zaxis_title="z",
                xaxis=dict(range=ranges[0]),
                yaxis=dict(range=ranges[1]),
                zaxis=dict(range=ranges[2]),
            ),
            width=800,  # Adjust figure size
            height=600,
        )
        fig.update_layout(showlegend=True)  # type: ignore
        fig.show()  # type: ignore # Display the interactive plot
