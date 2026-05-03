"""Build SITL binaries for ArduPilot."""

import subprocess
import sys
from pathlib import Path
from typing import cast

from simulator.config import ARDUPILOT_PATH
from simulator.helpers.ardupilot.types import VehicleInfoProtocol

_autotest = str(ARDUPILOT_PATH / "Tools" / "autotest")
if _autotest not in sys.path:
    sys.path.insert(0, _autotest)

from pysim import vehicleinfo  # type: ignore[import-untyped]  # noqa: E402

vinfo = cast(VehicleInfoProtocol, vehicleinfo.VehicleInfo())  # type: ignore[reportUnknownMemberType]


class Opts:
    """Options passed to vehicleinfo."""

    model = None
    build_target = None


def ensure_sitl_built(frame: str, firmware: str) -> Path:
    """
    Ensure the ArduPilot SITL binary for the given frame is built and return
    its path.
    """

    info = vinfo.options_for_frame(frame, firmware, Opts())
    waf_target = info["waf_target"]

    binary_name = waf_target.split("/")[-1]
    binary_path = ARDUPILOT_PATH / "build" / "sitl" / "bin" / binary_name

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

    info = vinfo.options_for_frame(frame, firmware, Opts())
    params = info["default_params_filename"]

    if isinstance(params, str):
        params = [params]

    return [str(ARDUPILOT_PATH / "Tools" / "autotest" / p) for p in params]
