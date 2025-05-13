"""
Configuration module for UAV-CYBER-SIM.

Defines system paths, base communication ports, and a color enum for UAV visualization.
"""

from enum import StrEnum, IntEnum
from pathlib import Path

# --- System Paths ---
HOME = Path.home()
QGC_PATH = HOME / "QGroundControl.AppImage"
QGC_INI_PATH = HOME / ".config" / "QGroundControl.org" / "QGroundControl.ini"
ARDUPILOT_VEHICLE_PATH = HOME / "ardupilot" / "Tools" / "autotest" / "sim_vehicle.py"
ARDUPILOT_GAZEBO_MODELS = HOME / "ardupilot_gazebo" / "models"

# --- Local Paths ---
LOGS_PATH = Path("ardupilot_logs").resolve()
PARAMS_PATH = Path("params/custom_params.parm").resolve()

# Ensure logs directory exists (can be cleaned later)
LOGS_PATH.mkdir(parents=True, exist_ok=True)


# --- Base Communication Ports ---
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

    QGC_TCP = 5763  # QGroundControl(TCP-no default option-uncomment in GCS module)
    QGC_UDP = 14550  # QGroundControl(UDP-default option-comment in GCS module)
    ARP = 14551  # Ardupilot Vehicle(UDP)
    GCS = 14552  # Ground Control Station(UDP)
    ORC = 14553  # Oracle(UDP)


# --- UAV Visualization Colors ---
class Color(StrEnum):
    """
    Enum for supported UAV marker colors in visualizations.
    """

    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    ORANGE = "orange"
    YELLOW = "yellow"

    def __str__(self) -> str:
        return self.value

    def __repr__(self) -> str:
        return self.value
