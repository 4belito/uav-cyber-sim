"""Runtime state for a UAV, including its active connection and processes."""

from __future__ import annotations

from dataclasses import dataclass, field
from subprocess import Popen
from typing import TYPE_CHECKING

from simulator.helpers.processes import SimProcess

if TYPE_CHECKING:
    from simulator.helpers.connections import MAVConnection


@dataclass
class VehicleRuntime:
    """
    Runtime object for a vehicle.

    `cmd_conn` is `None` for a GCS that only monitors this vehicle — a
    monitor never gets a command channel, only the owner does, so it has no
    way to send commands or drive an `Intervention` even in principle.
    """

    sysid: int
    conn: MAVConnection
    cmd_conn: MAVConnection | None
    processes: dict[SimProcess, Popen[bytes]] = field(
        default_factory=dict[SimProcess, Popen[bytes]]
    )
