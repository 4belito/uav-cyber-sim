"""Build SITL binaries for ArduPilot."""

import subprocess
import sys
from pathlib import Path
from typing import cast

from simulator.config import ARDUPILOT_PATH
from simulator.helpers.ardupilot.types import VehicleInfoProtocol

sys.path.append(str(ARDUPILOT_PATH / "Tools" / "autotest"))

from pysim import vehicleinfo  # type: ignore


class Opts:
    """Options passed to vehicleinfo."""

    model = None
    build_target = None


def ensure_sitl_built(frame: str, firmware: str) -> Path:
    """
    Ensure the ArduPilot SITL binary for the given frame is built and return
    its path.
    """
    vinfo = cast(VehicleInfoProtocol, vehicleinfo.VehicleInfo())  # type: ignore

    # 3. resolve target
    info = vinfo.options_for_frame(frame, firmware, Opts())
    waf_target: str = info["waf_target"]  # type: ignore

    binary_name = waf_target.split("/")[-1]
    binary_path = ARDUPILOT_PATH / "build" / "sitl" / "bin" / binary_name

    # 4. build if needed
    if binary_path.exists():
        return binary_path

    waf = ARDUPILOT_PATH / "modules" / "waf" / "waf-light"

    subprocess.run(
        [str(waf), "configure", "--board", "sitl"], cwd=ARDUPILOT_PATH, check=True
    )
    subprocess.run(
        [str(waf), "build", "--target", waf_target], cwd=ARDUPILOT_PATH, check=True
    )

    return binary_path


def get_default_params(frame: str, firmware: str) -> list[str]:
    """
    Get the default parameter files for a given frame and firmware.
    Returns absolute paths.
    """
    vinfo = cast(VehicleInfoProtocol, vehicleinfo.VehicleInfo())  # type: ignore

    info = vinfo.options_for_frame(frame, firmware, Opts())

    params: list[str] | str = info["default_params_filename"]  # type: ignore

    # normalize to list
    if isinstance(params, str):
        params = [params]

    # convert to absolute paths
    return [str(ARDUPILOT_PATH / "Tools" / "autotest" / p) for p in params]
