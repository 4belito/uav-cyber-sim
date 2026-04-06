"""
Configuration module for UAV-CYBER-SIM.

Defines system paths, base communication ports, and a color enum for UAV visualization.
"""

from enum import IntEnum, StrEnum
from pathlib import Path

# --- System Paths ---
ROOT = Path(__file__).parent
PROJECT_ROOT = ROOT.parent

QGC_PATH = PROJECT_ROOT / "QGroundControl" / "QGroundControl.AppImage"
ARDUPILOT_GAZEBO_MODELS = PROJECT_ROOT / "ardupilot_gazebo" / "models"
ARDUPILOT_PATH = PROJECT_ROOT / "ardupilot"


# --- Local Paths ---
ARDU_LOGS_PATH = ROOT / "ardupilot_logs"
LOGS_PATH = ROOT / "logs"
VEH_PARAMS_PATH = ROOT / "params" / "vehicle.parm"
SIM_PARAMS_PATH = ROOT / "params" / "simulation.py"
DATA_PATH = ROOT / "data"
RUNTIME_GAZEBO_MODELS = ROOT / "visualizer" / "gazebo" / "runtime_models"
RUNTIME_GAZEBO_WORLDS = ROOT / "visualizer" / "gazebo" / "runtime_worlds"
# Ensure logs directory exists (can be cleaned later)


class BasePort(IntEnum):
    """
    Base ports for QGroundControl (QGC), ArduPilot (ARP), Ground Control Station (GCS),
    and Oracle.

    - ARP ports increment by +10 per UAV instance.
    - GCS ports increment by +10 per GCS instance.
    - QGC uses a fixed UDP port (default: 14550).
    - Oracle uses fixed ports.

    All components except QGC connect to UAVLogic.
    QGC connects directly to ArduPilot (SITL).
    Gazebo connects to ArduPilot via UDP:
        - 9002 (to ArduPilot)
        - 9003 (from ArduPilot)

    Note:
    Using a different QGC UDP port requires code changes.

    """

    # ONE-PER-UAV PORTS
    ARP = 5760  # ArduPilot master port (TCP: PROXY->ARP)
    ARP2 = 5762  # ArduPilot SERIAL1 (TCP: auto-opened by SITL)
    ARP3 = 5763  # ArduPilot SERIAL2 (TCP: auto-opened by SITL)
    QGC = 14550  # QGroundControl UDP telemetry (SITL -> QGC)
    GCS = 14555  # Ground Control Station(UDP: LOGIC->GCS)
    RID_UP = 14556  # Remote ID (LOGIC->ORC)
    RID_DOWN = 14557  # Remote ID (ORC->LOGIC)
    ADSB_DOWN = 14558  # ORC -> ADSB injector (per UAV)

    # ONE-PER-GCS PORTS
    GCS_ZMQ = 30000  # GCS ZMQ (GCS->ORC)


# --- UAV Visualization Colors ---
class Color(StrEnum):
    """Enum for supported UAV marker colors in visualizations."""

    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    ORANGE = "orange"
    YELLOW = "yellow"
    BLACK = "black"
    WHITE = "white"

    def __str__(self) -> str:
        return self.value

    def __repr__(self) -> str:
        return self.value

    @property
    def emoji(self) -> str:
        """Return the emoji representation of the color."""
        return {
            Color.BLUE: "🟦",
            Color.GREEN: "🟩",
            Color.RED: "🟥",
            Color.ORANGE: "🟧",
            Color.YELLOW: "🟨",
            Color.BLACK: "⬛",
            Color.WHITE: "⬜",
        }[self]


Colors = list[Color]


# --- Environment Setup Commands ---
ENV_CMD_PYT = None
ENV_CMD_ARP = "source ~/.profile"
