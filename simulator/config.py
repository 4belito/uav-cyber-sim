"""
Configuration module for UAV-CYBER-SIM.

Defines system paths, base communication ports, and a color enum for UAV visualization.
"""

from enum import IntEnum, StrEnum
from pathlib import Path
from typing import Literal, cast

# --- System Paths ---
ROOT = Path(__file__).parent
PROJECT_ROOT = ROOT.parent

QGC_PATH = PROJECT_ROOT / "QGroundControl" / "QGroundControl.AppImage"
ARDUPILOT_GAZEBO_MODELS = PROJECT_ROOT / "ardupilot_gazebo" / "models"
ARDUPILOT_PATH = PROJECT_ROOT / "ardupilot"


# --- Local Paths ---
ARDU_LOGS_PATH = ROOT / "ardupilot_logs"
LOGS_PATH = ROOT / "logs"
PARAMS_PATH = ROOT / "params"
VEH_PARAMS_PATH = PARAMS_PATH / "vehicle.parm"
SIM_PARAMS_PATH = PARAMS_PATH / "simulation.py"
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
    ADSB = 5761  # ORC -> ADSB injector
    ARP2 = 5762  # ArduPilot SERIAL1 (TCP: auto-opened by SITL)
    ARP3 = 5763  # ArduPilot SERIAL2 (TCP: auto-opened by SITL)
    RID_UP = 5764  # Remote ID (LOGIC->ORC)
    RID_DOWN = 5765  # Remote ID (ORC->LOGIC)
    GCS = 5766  # Ground Control Station (UDP: LOGIC->GCS)
    GCS_CMD = 5768  # GCS command channel (UDP: GCS->LOGIC)
    # Man-in-the-middle interposition ports (only used when a MITM is enabled).
    # Placed well above the per-UAV comms cluster so they never collide with
    # another vehicle's block (safe up to a few hundred UAVs at +10 stride).
    MITM_TELEM = 9000  # MITM telemetry listener (UDP: LOGIC->MITM->GCS)
    MITM_CMD = 9001  # MITM command listener (UDP: GCS->MITM->LOGIC)

    # Universal Ports (fixed, not per UAV or GCS)
    ORC_DONE = 5767  # ZMQ ROUTER/DEALER (->ORC)
    QGC = 14550  # QGroundControl UDP telemetry DEFAULT port(UDP: SITL->QGC)


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


Firmware = Literal["ArduPlane", "ArduCopter"]


class Model(StrEnum):
    """Enum for supported UAV models in visualizations."""

    IRIS = "iris"
    ZEPHYR = "zephyr"

    @property
    def firmware(self) -> Firmware:
        """Return the corresponding ArduPilot firmware for the model."""
        return cast(
            Firmware,
            {
                Model.IRIS: "ArduCopter",
                Model.ZEPHYR: "ArduPlane",
            }[self],
        )

    def __call__(self, visualizer_name: str) -> str:
        """Return the corresponding Gazebo model name for the UAV model."""
        if visualizer_name.lower() == "gazebo":
            return "gazebo-" + self.value
        else:
            return {
                Model.IRIS: "copter-iris",
                Model.ZEPHYR: "plane-zephyr",
            }[self]


Colors = list[Color]


# --- Environment Setup Commands ---
ENV_CMD_PYT = None
ENV_CMD_ARP = "source ~/.profile"
