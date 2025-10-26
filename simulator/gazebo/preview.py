"""Visualization utilities for Gazebo trajectory rendering."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, TypeAlias

import numpy as np
import plotly.graph_objects as go
from numpy.typing import NDArray

from config import Color
from helpers.coordinates import ENU

SphereTriMesh: TypeAlias = tuple[
    NDArray[np.float64],
    NDArray[np.float64],
    NDArray[np.float64],
    list[int],
    list[int],
    list[int],
]


@dataclass
class GazMarker:
    """Visual waypoint with position, color, size, and transparency in Gazebo."""

    name: str
    group: str
    pos: ENU
    color: Color = Color.GREEN
    radius: float = 0.2
    alpha: float = 0.05


GazMarkers = list[GazMarker]


def show_markers(
    markers: Iterable[GazMarker],
    *,
    title: str = "Trajectories",
    frames: tuple[float, float, float] = (0.2, 0.2, 0.2),
    ground: float | None = 0,
) -> None:
    """Render waypoint markers in an interactive Plotly figure."""
    fig = create_trajectory_figure(markers, title, frames, ground)
    fig.show()  # type: ignore[call-arg]


def create_trajectory_figure(
    markers: Iterable[GazMarker],
    title: str,
    frames: tuple[float, float, float],
    ground: float | None,
) -> go.Figure:
    """Build a Plotly figure describing waypoint trajectories."""
    data, all_x, all_y, all_z = _build_plot_series(markers)
    ranges = _compute_ranges(all_x, all_y, all_z, frames, ground)
    fig: go.Figure = go.Figure(data)
    fig.update_layout(  # type: ignore[arg-type]
        title=dict(text=title, x=0.5, xanchor="center"),
        scene=dict(
            xaxis=dict(title="x", range=ranges[0]),
            yaxis=dict(title="y", range=ranges[1]),
            zaxis=dict(title="z", range=ranges[2]),
            aspectmode="data",
        ),
        width=800,
        height=600,
        showlegend=True,
        legend=dict(groupclick="togglegroup"),
    )
    return fig


def _compute_ranges(
    all_x: list[float],
    all_y: list[float],
    all_z: list[float],
    frames: tuple[float, float, float],
    ground: float | None,
) -> list[list[float]]:
    def scale(values: list[float], f: float) -> list[float]:
        vmin, vmax = min(values), max(values)
        margin = f * (vmax - vmin if vmax > vmin else 1.0)
        return [vmin - margin, vmax + margin]

    if not all_x:
        all_x = [0.0]
    if not all_y:
        all_y = [0.0]
    if not all_z:
        all_z = [0.0]

    x_range = scale(all_x, frames[0])
    y_range = scale(all_y, frames[1])
    z_range = scale(all_z, frames[2])
    if ground is not None:
        z_range[0] = ground
    return [x_range, y_range, z_range]


def _build_plot_series(
    marker_iter: Iterable[GazMarker],
) -> tuple[list[go.Mesh3d | go.Scatter3d], list[float], list[float], list[float]]:
    data: list[go.Mesh3d | go.Scatter3d] = []
    all_x: list[float] = []
    all_y: list[float] = []
    all_z: list[float] = []

    group_color: dict[str, str] = {}
    group_max_r: dict[str, float] = {}

    for mark in marker_iter:
        cx, cy, cz = float(mark.pos.x), float(mark.pos.y), float(mark.pos.z)
        r = float(mark.radius)
        color = mark.color.name.lower()
        opacity = 1.0 - float(mark.alpha)
        group = str(mark.group)

        nu = 22 if r >= 1.0 else 14
        nv = 32 if r >= 1.0 else 18
        x, y, z, i, j, k = _sphere_mesh(cx, cy, cz, r, nu=nu, nv=nv)
        data.append(
            go.Mesh3d(
                x=x,
                y=y,
                z=z,
                i=i,
                j=j,
                k=k,
                color=color,
                opacity=opacity,
                name=group,
                legendgroup=group,
                showlegend=False,
                flatshading=False,
                lighting=dict(ambient=0.35, diffuse=0.8, specular=0.2, roughness=0.9),
                lightposition=dict(x=200, y=200, z=100),
            )
        )

        group_color[group] = color
        group_max_r[group] = max(group_max_r.get(group, 0.0), r)

        all_x.extend([cx - r, cx + r])
        all_y.extend([cy - r, cy + r])
        all_z.extend([cz - r, cz + r])

    if group_max_r:
        desired_max_px = 18
        max_r = max(group_max_r.values())
        px_per_unit = desired_max_px / max_r if max_r > 0 else 1.0
        min_px = 6

        for group, color in group_color.items():
            r = group_max_r[group]
            size_px = max(min_px, r * px_per_unit)
            data.append(
                go.Scatter3d(
                    x=[0],
                    y=[0],
                    z=[0],
                    mode="markers",
                    name=group,
                    legendgroup=group,
                    showlegend=True,
                    visible="legendonly",
                    hoverinfo="skip",
                    marker=dict(
                        symbol="circle", size=size_px, color=color, opacity=1.0
                    ),
                )
            )

    return data, all_x, all_y, all_z


def _sphere_mesh(
    cx: float, cy: float, cz: float, r: float, nu: int = 16, nv: int = 24
) -> SphereTriMesh:
    """Triangulate a sphere centered at (cx, cy, cz) with radius r."""
    if r <= 0:
        return np.array([cx]), np.array([cy]), np.array([cz]), [], [], []

    u = np.linspace(0.0, np.pi, nu)
    v = np.linspace(0.0, 2.0 * np.pi, nv, endpoint=False)
    uu, vv = np.meshgrid(u, v, indexing="ij")

    x = cx + r * np.sin(uu) * np.cos(vv)
    y = cy + r * np.sin(uu) * np.sin(vv)
    z = cz + r * np.cos(uu)

    i: list[int] = []
    j: list[int] = []
    k: list[int] = []
    for a in range(nu - 1):
        for b in range(nv):
            b2 = (b + 1) % nv
            p00 = a * nv + b
            p01 = a * nv + b2
            p10 = (a + 1) * nv + b
            p11 = (a + 1) * nv + b2
            i += [p00, p11]
            j += [p10, p01]
            k += [p11, p00]

    return x.ravel(), y.ravel(), z.ravel(), i, j, k
