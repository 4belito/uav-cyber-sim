"""Runtime state for a UAV, including its active connection and processes."""

from dataclasses import dataclass, field
from subprocess import Popen

from simulator.helpers.connections import MAVConnection
from simulator.helpers.processes import SimProcess


@dataclass
class VehicleRuntime:
    """Runtime object for a vehicle."""

    sysid: int
    conn: MAVConnection
    cmd_conn: MAVConnection
    processes: dict[SimProcess, Popen[bytes]] = field(
        default_factory=dict[SimProcess, Popen[bytes]]
    )
