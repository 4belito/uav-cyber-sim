"""Read ground-truth trajectories back from the per-vehicle JSONL logs."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from simulator.helpers.coordinates import GRA, ENUs

if TYPE_CHECKING:
    from pathlib import Path


def read_true_track(msgs_dir: Path, sysid: int, gra_origin: GRA) -> ENUs:
    """
    Reconstruct a vehicle's real ENU trajectory from its MAVLink log.

    The Oracle only ever receives Remote ID, which an attacker spoofs, so its
    live track is the *transmitted* position. The **real** position is recorded
    independently by ``MAVLinkManager`` as ``mavlink_in`` ``GLOBAL_POSITION_INT``
    records in ``msgs_dir / f"veh_{sysid}.jsonl"``. This reads those back and
    converts them to the local ENU frame exactly as ``RIDManager._build_rid``
    does, so the result lines up with the Oracle's own tracks.

    Returns an empty list if the log file does not exist (e.g. the vehicle never
    ran, or the logs were cleaned).
    """
    log_path = msgs_dir / f"veh_{sysid}.jsonl"
    if not log_path.exists():
        return []

    track: ENUs = []
    with log_path.open() as f:
        for line in f:
            record = json.loads(line)
            if (
                record.get("type") != "mavlink_in"
                or record.get("msg_type") != "GLOBAL_POSITION_INT"
            ):
                continue
            data = record["data"]
            lat = data["lat"]
            lon = data["lon"]
            # lat/lon 0,0 == no EKF fix yet, not a position (Oracle drops these too).
            if lat == 0 and lon == 0:
                continue
            gra = GRA.from_global_int(lat, lon, data["alt"])
            track.append(gra_origin.to_rel(gra))
    return track
