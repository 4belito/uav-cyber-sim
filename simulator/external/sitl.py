"""Build SITL binaries for ArduPilot."""

import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import cast

from simulator.config import ARDUPILOT_PATH, Firmware
from simulator.helpers.ardupilot.types import FrameOptions, VehicleInfoProtocol

_autotest = str(ARDUPILOT_PATH / "Tools" / "autotest")
if _autotest not in sys.path:
    sys.path.insert(0, _autotest)

from pysim import vehicleinfo  # type: ignore[import-untyped]  # noqa: E402

_vinfo = cast(VehicleInfoProtocol, vehicleinfo.VehicleInfo())  # type: ignore[reportUnknownMemberType]
_opts = SimpleNamespace(model=None, build_target=None)


def _romfs_json_newer_than(binary_path: Path) -> bool:
    """Return True if any ROMFS JSON model file is newer than the binary.

    ROMFS data is compiled into the binary at build time. A newer JSON file
    means the binary must be rebuilt so the updated model is embedded.
    """
    models_dir = ARDUPILOT_PATH / "Tools" / "autotest" / "models"
    binary_mtime = binary_path.stat().st_mtime
    return any(
        json_file.stat().st_mtime > binary_mtime
        for json_file in models_dir.glob("*.json")
    )


def ensure_sitl_built(frame: str, firmware: Firmware) -> Path:
    """Ensure the SITL binary for the given frame is built and return its path."""
    info = _vinfo.options_for_frame(frame, firmware, _opts)
    binary_name = info["waf_target"].split("/")[-1]
    binary_path = ARDUPILOT_PATH / "build" / "sitl" / "bin" / binary_name

    if binary_path.exists() and not _romfs_json_newer_than(binary_path):
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


def get_frame_info(frame: str, firmware: Firmware) -> FrameOptions:
    """Return vehicleinfo options for the given frame and firmware."""
    return _vinfo.options_for_frame(frame, firmware, _opts)


def get_default_params(info: FrameOptions) -> list[str]:
    """Return absolute paths to the default parameter files from a FrameOptions dict."""
    params = info["default_params_filename"]
    if isinstance(params, str):
        params = [params]
    return [str(ARDUPILOT_PATH / "Tools" / "autotest" / p) for p in params]


def resolve_sitl_build(frame: str, firmware: Firmware) -> tuple[Path, str, list[str]]:
    binary = ensure_sitl_built(frame, firmware)
    frame_info = get_frame_info(frame, firmware)
    return binary, frame_info["model"], get_default_params(frame_info)
