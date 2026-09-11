"""TypedDict schemas for logic runtime configuration."""

from __future__ import annotations

from typing import Any, NotRequired, TypedDict


class LogicConfig(TypedDict):
    """
    UAV logic-process configuration.

    Notable optional fields:

    - `gcs_telem_ports` — UDP ports of every GCS monitoring this vehicle. Empty
      when the vehicle is unmonitored, in which case no telemetry is emitted and
      no GCS ack is awaited.
    - `rid_frequency` — Remote ID broadcast rate in Hz, set on the Oracle.
    - `spoof` — a serialized `SpoofProfile` for a spoofing vehicle; absent/None
      for an honest one (see `simulator.entities.spoof_profile`).
    """

    sysid: int
    veh_port_offset: int
    oracle_port_offset: int
    gra_origin_dict: dict[str, float]
    plan_spec: dict[str, Any]
    home_heading: NotRequired[float]
    mitm: NotRequired[bool]
    gcs_telem_ports: NotRequired[list[int]]
    rid_frequency: NotRequired[int]
    spoof: NotRequired[dict[str, Any] | None]
