"""JSONL logger for structured data collection."""

from __future__ import annotations

import json
import logging
import time
from typing import TYPE_CHECKING

from simulator.helpers.connections.mavlink.streams import JSONType, make_json_safe

if TYPE_CHECKING:
    from pathlib import Path

Record = dict[str, JSONType]


class DataLogger:
    """Reusable JSONL logger for UAV data."""

    def __init__(self, path: Path, sysid: int) -> None:
        path.mkdir(parents=True, exist_ok=True)
        self.sysid = sysid
        self._path = path / f"veh_{sysid}.jsonl"

    def write(self, record: Record) -> None:
        """Write a record to the JSONL file with error handling."""
        try:
            record = {
                "sysid": self.sysid,
                "time_logged": time.time(),
                **record,
            }

            safe = make_json_safe(record)
            with self._path.open("a") as f:
                f.write(json.dumps(safe) + "\n")

        except Exception as e:
            logging.error(f"Data write error: {e}")
