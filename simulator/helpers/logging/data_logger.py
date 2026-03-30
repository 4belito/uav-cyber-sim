"""JSONL logger for structured data collection."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path

from simulator.helpers.connections.mavlink.streams import JSONType, make_json_safe

Record = dict[str, JSONType]


class DataLogger:
    """Reusable JSONL logger for UAV data."""

    def __init__(self, path: Path, sysid: int) -> None:
        path.mkdir(parents=True, exist_ok=True)
        self.sysid = sysid
        self._file = open(path / f"veh_{sysid}.jsonl", "a")

    def write(self, record: Record) -> None:
        """Write a record to the JSONL file with error handling."""
        try:
            record = {
                "sysid": self.sysid,
                "time_logged": time.time(),
                **record,
            }

            safe = make_json_safe(record)
            self._file.write(json.dumps(safe) + "\n")
            self._file.flush()

        except Exception as e:
            logging.error(f"Data write error: {e}")

    def close(self) -> None:
        """Close the JSONL file. Should be called when done logging."""
        self._file.close()
