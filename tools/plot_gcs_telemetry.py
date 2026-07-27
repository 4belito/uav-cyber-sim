"""Plot per-drone flight paths (longitude vs latitude) from GCS telemetry logs.

The GCS writes one JSONL file per drone (``veh_<sysid>.jsonl``) under
``simulator/data/gcs_msgs/``. Each line is a received MAVLink message; this
script pulls the ``GLOBAL_POSITION_INT`` records, converts the integer lat/lon
(1e7 degrees) to decimal degrees, and plots every drone's trajectory on a
single longitude-vs-latitude graph (a top-down map view of the whole run).

Usage::

    python -m tools.plot_gcs_telemetry                  # read default dir, show
    python -m tools.plot_gcs_telemetry --save out.png   # write to file instead
    python -m tools.plot_gcs_telemetry --data-dir <dir> # custom log directory
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

from simulator.config import DATA_PATH

DEFAULT_DATA_DIR = DATA_PATH / "gcs_msgs"


@dataclass
class Track:
    """A single drone's position samples in decimal degrees, in time order."""

    sysid: int
    lats: list[float]
    lons: list[float]


def load_track(path: Path) -> Track | None:
    """Read one ``veh_<sysid>.jsonl`` file into a :class:`Track`.

    Records with a zero lat/lon (emitted before the SITL GPS fix) are skipped.
    Returns ``None`` if the file holds no usable position fixes.
    """
    sysid = int(path.stem.split("_")[-1])
    lats: list[float] = []
    lons: list[float] = []

    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            if record.get("msg_type") != "GLOBAL_POSITION_INT":
                continue
            data = record["data"]
            lat_raw = data["lat"]
            lon_raw = data["lon"]
            if lat_raw == 0 and lon_raw == 0:
                continue
            lats.append(lat_raw / 1e7)
            lons.append(lon_raw / 1e7)

    if not lats:
        return None
    return Track(sysid=sysid, lats=lats, lons=lons)


def load_tracks(data_dir: Path) -> list[Track]:
    """Load every ``veh_*.jsonl`` track in ``data_dir``, sorted by sysid."""
    tracks: list[Track] = []
    for path in sorted(data_dir.glob("veh_*.jsonl")):
        track = load_track(path)
        if track is not None:
            tracks.append(track)
    return sorted(tracks, key=lambda t: t.sysid)


def plot_tracks(tracks: list[Track], save: Path | None) -> None:
    """Plot all drone trajectories on one longitude-vs-latitude graph."""
    if not tracks:
        raise SystemExit("No drone position data found to plot.")

    fig, ax = plt.subplots(figsize=(9, 9))

    for track in tracks:
        label = f"Drone {track.sysid}"
        (line,) = ax.plot(track.lons, track.lats, marker=".", markersize=2, label=label)
        color = line.get_color()
        # Mark where each path starts (○) and ends (■).
        ax.scatter(
            track.lons[0], track.lats[0],
            color=color, marker="o", s=70, edgecolors="black", zorder=3,
        )
        ax.scatter(
            track.lons[-1], track.lats[-1],
            color=color, marker="s", s=70, edgecolors="black", zorder=3,
        )

    ax.set_xlabel("Longitude (°)")
    ax.set_ylabel("Latitude (°)")
    ax.set_title("GCS-received drone trajectories (○ start, ■ end)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    # Equal scaling so the paths aren't visually distorted.
    ax.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()

    if save is not None:
        fig.savefig(save, dpi=150)
        print(f"Saved plot to {save}")
    else:
        plt.show()


def parse_arguments() -> tuple[Path, Path | None]:
    """Parse CLI arguments: telemetry directory and optional output image."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help=f"Directory of veh_<sysid>.jsonl logs (default: {DEFAULT_DATA_DIR})",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Write the figure to this path instead of showing it interactively.",
    )
    args = parser.parse_args()
    return args.data_dir, args.save


def main() -> None:
    """Entry point: load tracks from the telemetry directory and plot them."""
    data_dir, save = parse_arguments()
    tracks = load_tracks(data_dir)
    plot_tracks(tracks, save)


if __name__ == "__main__":
    main()
