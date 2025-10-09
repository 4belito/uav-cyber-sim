"""
Configuration module for UAV-CYBER-SIM.

Defines system paths, base communication ports, and a color enum for UAV visualization.
"""

from enum import IntEnum, StrEnum
from pathlib import Path

from params.simulation import CONNECT_GCS_TO_ARP

# --- System Paths ---
HOME = Path.home()
QGC_PATH = HOME / "QGroundControl.AppImage"
QGC_INI_PATH = HOME / ".config" / "QGroundControl.org" / "QGroundControl Daily.ini"
ARDUPILOT_VEHICLE_PATH = HOME / "ardupilot" / "Tools" / "autotest" / "sim_vehicle.py"
ARDUPILOT_GAZEBO_MODELS = HOME / "ardupilot_gazebo" / "models"

# --- Local Paths ---
ROOT = Path(__file__).parent
ARDU_LOGS_PATH = (ROOT / "ardupilot_logs").resolve()
LOGS_PATH = (ROOT / "logs").resolve()
VEH_PARAMS_PATH = (ROOT / "params/vehicle.parm").resolve()
SIM_PARAMS_PATH = (ROOT / "params/simulation.py").resolve()
DATA_PATH = (ROOT / "data").resolve()

# Ensure logs directory exists (can be cleaned later)
ARDU_LOGS_PATH.mkdir(parents=True, exist_ok=True)


# --- Base Communication Ports ---
QGC_UDP = 14550  # QGroundControl(UDP-default option-connect to the proxy)
QGC_TCP = 5762  # QGroundControl(TCP-no default-connect to Ardupilot-like Gazebo)


class BasePort(IntEnum):
    """
    Base ports for QGrounfControl(QGC), ArduPilot (ARP), Ground control Station (GCS),
    and Oracle.

    - QGC and ARP ports increment by +10 per UAV instance.
    - GCS ports increment by +10 per GCS instance.
    - Oracle uses a fixed port.

    All components except QGC connect to the UAVLogic.
    QGC connects directly to ArduPilot (SITL).
    Gazebo connects to ArduPilot via UDP 9002 (to ArduPilot) and 9003 (from ArduPilot).
    """

    # ONE-PER-UAV PORTS
    # variable QGC_UDP is not actually being used because QGround control connects
    # automatically to UDP 14550
    QGC = QGC_TCP if CONNECT_GCS_TO_ARP else QGC_UDP
    ARP = 5760  # Ardupilot Vehicle(TCP: PROXY->ARP)
    LOG = 14551  # Vehicle(TCP: PROXY->LOGIC)
    GCS = 14552  # Ground Control Station(UDP: LOGIC->GCS)
    RID_UP = 14554  # Remote ID (LOGIC->ORC)
    RID_DOWN = 14555  # Remote ID (ORC->LOGIC)
    RID_DATA = 14556  # Remote ID (PROXY->LOGIC) internal

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
ENV_CMD_GAZ = "source ~/.profile"
