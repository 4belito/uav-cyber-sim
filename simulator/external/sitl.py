"""Build SITL binaries for ArduPilot."""

import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import cast

from simulator.config import ARDUPILOT_PATH
from simulator.helpers.ardupilot.types import FrameOptions, VehicleInfoProtocol

_autotest = str(ARDUPILOT_PATH / "Tools" / "autotest")
if _autotest not in sys.path:
    sys.path.insert(0, _autotest)

from pysim import vehicleinfo  # type: ignore[import-untyped]  # noqa: E402

_vinfo = cast(VehicleInfoProtocol, vehicleinfo.VehicleInfo())  # type: ignore[reportUnknownMemberType]
_opts = SimpleNamespace(model=None, build_target=None)


def ensure_sitl_built(frame: str, firmware: str) -> Path:
    """Ensure the SITL binary for the given frame is built and return its path."""
    info = _vinfo.options_for_frame(frame, firmware, _opts)
    binary_name = info["waf_target"].split("/")[-1]
    binary_path = ARDUPILOT_PATH / "build" / "sitl" / "bin" / binary_name

    if binary_path.exists():
        return binary_path

    waf = ARDUPILOT_PATH / "modules" / "waf" / "waf-light"
    subprocess.run(
        [str(waf), "configure", "--board", "sitl"], cwd=ARDUPILOT_PATH, check=True
    )
    subprocess.run(
        [str(waf), "build", "--target", info["waf_target"]],
        cwd=ARDUPILOT_PATH,
        check=True,
    )
    return binary_path


def get_frame_info(frame: str, firmware: str) -> FrameOptions:
    """Return vehicleinfo options for the given frame and firmware."""
    return _vinfo.options_for_frame(frame, firmware, _opts)


def get_default_params(info: FrameOptions) -> list[str]:
    """Return absolute paths to the default parameter files from a FrameOptions dict."""
    params = info["default_params_filename"]
    if isinstance(params, str):
        params = [params]
    return [str(ARDUPILOT_PATH / "Tools" / "autotest" / p) for p in params]
