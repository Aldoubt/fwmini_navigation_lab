"""Pure Python, fail-closed launch contracts; no ROS imports."""

import json
from pathlib import Path


class ContractError(ValueError):
    """A backend or hardware combination is not eligible to launch."""


BACKENDS = {"rtabmap", "slam_toolbox", "cartographer_2d", "fastlio2"}
SOURCES = {"auto", "none", "depth", "cloud"}


def resolve_scan_source(backend, requested):
    if backend not in BACKENDS:
        raise ContractError("Unknown backend: " + backend)
    if requested not in SOURCES:
        raise ContractError("Unknown scan source: " + requested)
    if requested == "auto":
        return "none" if backend in ("rtabmap", "fastlio2") else "depth"
    return requested


def validate_selection(backend, mode, source, database_path="", posegraph_file=""):
    source = resolve_scan_source(backend, source)
    if mode not in ("mapping", "localization"):
        raise ContractError("mode must be mapping/localization")
    if backend == "fastlio2":
        raise ContractError("FAST-LIO2 requires unverified MID360/IMU/TF/global-map adapters")
    if backend in ("cartographer_2d", "slam_toolbox") and source == "none":
        raise ContractError("2D backends require a /scan source")
    if backend == "rtabmap" and source != "none":
        raise ContractError("RGB-D RTAB-Map profile requires scan_source:=none")
    if mode == "localization":
        if backend == "cartographer_2d":
            raise ContractError("Cartographer .pbstream pure-localization flow not tested")
        filename = database_path if backend == "rtabmap" else posegraph_file
        if not filename or not Path(filename).expanduser().is_file():
            raise ContractError("Localization needs existing native map asset: " + filename)
    return source


def validate_hardware_contract(path, backend, source):
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise ContractError("Hardware verification JSON missing: " + str(file_path))
    try:
        data = json.loads(file_path.read_text(encoding="utf8"))
    except (OSError, ValueError) as exc:
        raise ContractError("Invalid hardware verification JSON: " + str(exc)) from exc
    required = ["wheel_odom_verified", "footprint_verified", "cmd_vel_verified"]
    if backend == "rtabmap" or source == "depth":
        required.append("camera_extrinsic_verified")
    else:
        required.append("lidar_extrinsic_verified")
    missing = [k for k in required if data.get(k) is not True]
    if data.get("odom_frame") != "odom" or data.get("base_frame") != "base_link":
        missing.append("frame convention odom/base_link")
    if missing:
        raise ContractError("Navigation BLOCKED: " + ", ".join(missing))
    return True
