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


class VehPort(IntEnum):
    """
    Per-vehicle base ports: a vehicle binds exactly **one** port per member,
    at `base + its own offset`. For a per-vehicle *window* of ports see
    `GCS_TELEM_WINDOW` below.

    The whole set is one contiguous run no wider than the stride, so a vehicle's
    ports are exactly its own block and can never reach into the next one's.

    All components except QGC connect to UAVLogic; QGC connects directly to
    ArduPilot (SITL).

    SITL also listens on `base_port + N` for each serial it leaves as TCP
    (`_serial_path[]` in `AP_HAL_SITL/SITL_State.h`), so the block is shared
    with ArduPilot and the split below is load-bearing: **our TCP users sit only
    on slots SITL leaves free, and the slots SITL does use we take as UDP** —
    a UDP bind and a TCP listen on the same number do not conflict.

    ```text
    slot | ours                  | SITL serial (TCP)
     +0  | ARP           tcp     | SERIAL0
     +1  | ADSB/zmq      tcp     | -
     +2  | ARP2          tcp     | SERIAL1
     +3  | ARP3          tcp     | SERIAL2
     +4  | RID_UP/zmq    tcp     | -
     +5  | RID_DOWN/zmq  tcp     | SERIAL5 -> freed by --serial5=uart:
     +6  | GCS_CMD       udp     | SERIAL6
     +7  | MITM_TELEM    udp     | SERIAL7
     +8  | MITM_CMD      udp     | SERIAL8
     +9  | (spare)               | -
    ```

    Moving one of the ZMQ ports (+1/+4/+5) onto a TCP serial slot, or any of
    +6/+7/+8 to a TCP transport, would collide with SITL.
    """

    ARP = 5760  # ArduPilot master port (TCP: PROXY->ARP)
    ADSB = 5761  # ORC -> ADSB injector
    ARP2 = 5762  # ArduPilot SERIAL1 (TCP: auto-opened by SITL)
    ARP3 = 5763  # ArduPilot SERIAL2 (TCP: auto-opened by SITL)
    RID_UP = 5764  # Remote ID (LOGIC->ORC)
    RID_DOWN = 5765  # Remote ID (ORC->LOGIC)
    GCS_CMD = 5766  # GCS command channel (UDP: any GCS->LOGIC)
    # Only bound when a MITM is enabled, but always reserved with the block.
    MITM_TELEM = 5767  # MITM telemetry listener (UDP: LOGIC->MITM->GCS)
    MITM_CMD = 5768  # MITM command listener (UDP: GCS->MITM->LOGIC)
    # 5769 spare


class SitlPort(IntEnum):
    """
    Ports ArduPilot SITL claims per vehicle by itself — all UDP, none ours.

    `-I N` adds `10 * N` to each while it is left at its default
    (`AP_HAL_SITL/SITL_cmdline.cpp`), and we override none of them, so they
    stride exactly like `VehPort` and must be reserved with the vehicle's block
    even though nothing of ours ever binds them.
    """

    RCIN = 5501  # RC input listener
    FG_VIEW = 5503  # FlightGear view output
    SIM_OUT = 9002  # physics backend (Gazebo): SITL -> sim
    SIM_IN = 9003  # physics backend (Gazebo): sim -> SITL
    IRLOCK = 9005  # IR-Lock sensor listener


# ArduPilot's instance stride, mirrored here — NOT a setting of ours.
#
# Everything downstream is forced by it: vehicle *n* takes offset
# `n * SITL_INSTANCE_STRIDE`, owning `base + offset` at every VehPort plus a
# telemetry window exactly this wide. ArduPilot hardcodes the number: `-I N`
# "adds 10*instance to all port numbers" (AP_HAL_SITL/SITL_cmdline.cpp). We feed
# a vehicle's offset straight back to SITL as `-I offset // stride`, and SITL
# then places every `SitlPort` at `default + 10 * instance` no matter what is set
# here — so any other value silently reserves the wrong ports (with 20, vehicle 1
# binds RCIN 5511 while we reserve 5521, leaving the real one unprotected).
# Changing it means changing ArduPilot.
SITL_INSTANCE_STRIDE = 10

# Per-vehicle port *window* — unlike `VehPort`, one vehicle takes several ports
# here: its k-th GCS listens for telemetry on `GCS_TELEM_WINDOW + k + offset`.
# The window is one stride wide, which is therefore also the ceiling on how many
# GCSs may monitor a single vehicle.
GCS_TELEM_WINDOW = 20000

# A vehicle's block must fit inside its own stride. Otherwise vehicle n's ports
# reach into vehicle n+1's, and since allocation only probes (nothing is bound
# yet) both would be handed the same port and collide at runtime.
if max(VehPort) - min(VehPort) >= SITL_INSTANCE_STRIDE:
    raise ValueError(
        f"VehPort spans {max(VehPort) - min(VehPort) + 1} ports "
        f"({min(VehPort)}-{max(VehPort)}) but the stride is "
        f"{SITL_INSTANCE_STRIDE}; "
        f"widen SITL_INSTANCE_STRIDE or the vehicle blocks will overlap."
    )


class SimPort(IntEnum):
    """
    Simulation-wide ports: one per run, never multiplied per vehicle.

    Kept clear of every `VehPort` range so the two never interleave. ORC_DONE
    is still searched (stride `SITL_INSTANCE_STRIDE`) if a port is taken, and that
    search excludes the vehicle blocks explicitly.
    """

    QGC = 14550  # QGroundControl UDP telemetry DEFAULT port (UDP: SITL->QGC)
    ORC_DONE = 14560  # ZMQ ROUTER/DEALER (->ORC)


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
