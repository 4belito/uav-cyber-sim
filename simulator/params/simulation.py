"""
MAVLink link plumbing, shared by the vehicle logic and the GCS.

These are protocol rates that no scenario varies. Anything that *is* a scenario
or run choice is a constructor argument instead:

* `Oracle(transmission_range=..., rid_frequency=..., record_positions=...,
  network_sim=...)` — the Remote ID model and what the Oracle records.
* `Simulator(speedup=...)` — the SITL wall-clock multiplier.
"""

from __future__ import annotations

HEARTBEAT_FREQUENCY: int = 1  # Hz - logic and GCS
DATA_STREAM_FREQUENCY: int = 5  # Hz - logic
