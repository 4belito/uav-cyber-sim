"""
Configuration module for UAV-CYBER-SIM.

Defines system paths, base communication ports, and a color enum for UAV visualization.
"""

from enum import Enum
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
VEH_BASE_PORT = 1551
GCS_BASE_PORT = 1552
ORC_BASE_PORT = 1553


# --- UAV Visualization Colors ---
class Color(str, Enum):
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
