"""Gazebo configuration class."""

from dataclasses import dataclass

import plotly.graph_objects as go  # type: ignore

from config import Color
from helpers.change_coordinates import pose, poses
from mavlink.customtypes.location import ENU, ENUPose, ENUPoses, ENUs

COLOR_MAP: dict[Color, str] = {
    Color.BLUE: "0.0 0.0 1.0 1",
    Color.GREEN: "0.306 0.604 0.024 1",
    Color.RED: "0.8 0.0 0.0 1",
    Color.ORANGE: "1.0 0.5 0.0 1",
    Color.YELLOW: "1.0 1.0 0.0 1",
    Color.WHITE: "1.0 1.0 1.0 1",
}


@dataclass
class GazWP:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    pos: ENU
    color: Color = Color.GREEN
    radius: float = 0.2
    alpha: float = 0.05


GazTraj = list[GazWP]


@dataclass
class GazVehicle:
    """Represents a vehicle with a model and a trajectory."""

    model: str
    color: Color
    home: ENUPose
    mtraj: GazTraj


GazVehicles = list[GazVehicle]


class ConfigGazebo:
    """Gazebo configuration and marker visualization manager."""

    def __init__(
        self,
        origin: ENUPose,
        world_path: str,
    ) -> None:
        self.origin = origin
        self.world_path = world_path
        self.vehicles: GazVehicles = []

    def add_vehicle(
        self,
        mtraj: GazTraj,
        home: ENUPose,
        color: Color = Color.BLUE,
        model: str = "iris",
    ) -> None:
        """Add a vehicle to the Gazebo configuration."""
        self.vehicles.append(
            GazVehicle(
                model=model,
                color=color,
                home=home,
                mtraj=mtraj,
            )
        )

    def add(
        self,
        base_path: ENUs | ENUPoses,
        base_home: ENUPose,
        color: Color = Color.BLUE,
        model: str = "iris",
    ) -> None:
        """Shortcut to add a vehicle from a raw path."""
        home_path = poses(base_home, base_path)
        path = poses(self.origin, home_path)
        home = pose(self.origin, base_home)
        mtraj = ConfigGazebo.create_mtraj(traj=path, color=color)
        self.add_vehicle(mtraj=mtraj, color=color, model=model, home=home)

    def remove_vehicle_at(self, index: int) -> bool:
        """Remove a vehicle by index."""
        if 0 <= index < len(self.vehicles):
            del self.vehicles[index]
            return True
        return False

    def show(
        self,
        title: str = "Gazebo GazVehicle Trajectories",
        origin_color: Color = Color.WHITE,
        frames: tuple[float, float, float] = (0.2, 0.2, 0.2),
        ground: float | None = 0,
    ) -> None:
        """Render a 3D interactive plot of waypoint trajectories using Plotly."""
        data, all_x, all_y, all_z = self._extract_plot_data()

        # Add origin marker
        ox, oy, oz, _ = self.origin
        data.append(self._make_point(ENU(ox, oy, oz), "origin", origin_color))
        all_x.append(ox)
        all_y.append(oy)
        all_z.append(oz)

        ranges = self._compute_ranges(all_x, all_y, all_z, frames, ground)
        fig: go.Figure = go.Figure(data)
        fig.update_layout(  # type: ignore
            title=dict(text=title, x=0.5, xanchor="center"),
            scene=dict(
                xaxis=dict(title="x", range=ranges[0]),
                yaxis=dict(title="y", range=ranges[1]),
                zaxis=dict(title="z", range=ranges[2]),
            ),
            width=800,
            height=600,
            showlegend=True,
        )
        fig.show()  # type: ignore

    def _extract_plot_data(
        self,
    ) -> tuple[list[go.Scatter3d], list[float], list[float], list[float]]:
        data: list[go.Scatter3d] = []
        all_x: list[float] = []
        all_y: list[float] = []
        all_z: list[float] = []

        for i, veh in enumerate(self.vehicles):
            if not veh.mtraj:
                continue
            pos_color = ((w.pos[0], w.pos[1], w.pos[2], w.color) for w in veh.mtraj)
            xs, ys, zs, colors = map(list, zip(*pos_color))
            trace = self._make_traj(xs, ys, zs, colors, name=f"trajectory {i}")
            data.append(trace)
            all_x += xs
            all_y += ys
            all_z += zs

        return data, all_x, all_y, all_z

    def _make_traj(
        self,
        xs: list[float],
        ys: list[float],
        zs: list[float],
        colors: list[Color],
        name: str,
        size: int = 6,
    ) -> go.Scatter3d:
        """Create a 3D scatter trace from trajectory coordinates and colors."""
        return go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="markers",
            marker=dict(size=size, color=[c.name.lower() for c in colors]),
            name=name,
        )

    def _make_point(
        self,
        pos: ENU,
        label: str,
        color: Color = Color.BLACK,
        size: int = 8,
    ) -> go.Scatter3d:
        """Create a labeled point marker for the 3D plot."""
        return go.Scatter3d(
            x=[pos[0]],
            y=[pos[1]],
            z=[pos[2]],
            mode="markers+text",
            marker=dict(size=size, color=color.name.lower()),
            text=[label],
            textposition="top center",
            name=label,
        )

    def _compute_ranges(
        self,
        all_x: list[float],
        all_y: list[float],
        all_z: list[float],
        frames: tuple[float, float, float],
        ground: float | None,
    ) -> list[list[float]]:
        def scale(values: list[float], f: float) -> list[float]:
            vmin, vmax = min(values), max(values)
            margin = f * (vmax - vmin)
            return [vmin - margin, vmax + margin]

        x_range = scale(all_x, frames[0])
        y_range = scale(all_y, frames[1])
        z_range = scale(all_z, frames[2])
        if ground is not None:
            z_range[0] = ground
        return [x_range, y_range, z_range]

    def __str__(self) -> str:
        lines = [
            "ConfigGazebo:",
            f"  world_path: {self.world_path}",
            f"  origin: {self.origin}",
            f"  vehicles ({len(self.vehicles)}):",
        ]
        for v in self.vehicles:
            lines.append(f"    - model: {v.model}")
            lines.append(f"      color: {v.color.name}")
            lines.append(f"      trajectory ({len(v.mtraj)} waypoints):")
            for wp in v.mtraj:
                lines.append(f"        {wp}")
        return "\n".join(lines)

    @staticmethod
    def create_mtraj(
        traj: ENUs | ENUPoses,
        color: Color = Color.GREEN,
        radius: float = 0.2,
        alpha: float = 0.05,
    ) -> GazTraj:
        """Create a trajectory from waypoints as GazWPMarker objects."""
        markertraj: GazTraj = []
        for pos in traj:
            markertraj.append(
                GazWP(
                    pos=ENU(*pos[:3]),
                    color=color,
                    radius=radius,
                    alpha=alpha,
                )
            )
        return markertraj
