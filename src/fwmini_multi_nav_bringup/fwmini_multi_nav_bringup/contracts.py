1: """Pure Python, fail-closed launch contracts; no ROS imports."""
2: 
3: import json
4: from pathlib import Path
5: 
6: 
7: class ContractError(ValueError):
8:     """A backend or hardware combination is not eligible to launch."""
9: 
10: 
11: BACKENDS = {"rtabmap", "slam_toolbox", "cartographer_2d", "fastlio2"}
12: SOURCES = {"auto", "none", "depth", "cloud"}
13: 
14: 
15: def resolve_scan_source(backend, requested):
16:     if backend not in BACKENDS:
17:         raise ContractError("Unknown backend: " + backend)
18:     if requested not in SOURCES:
19:         raise ContractError("Unknown scan source: " + requested)
20:     if requested == "auto":
21:         return "none" if backend in ("rtabmap", "fastlio2") else "depth"
22:     return requested
23: 
24: 
25: def validate_selection(backend, mode, source, database_path="", posegraph_file=""):
26:     source = resolve_scan_source(backend, source)
27:     if mode not in ("mapping", "localization"):
28:         raise ContractError("mode must be mapping/localization")
29:     if backend == "fastlio2":
30:         raise ContractError("FAST-LIO2 requires unverified MID360/IMU/TF/global-map adapters")
31:     if backend in ("cartographer_2d", "slam_toolbox") and source == "none":
32:         raise ContractError("2D backends require a /scan source")
33:     if backend == "rtabmap" and source != "none":
34:         raise ContractError("RGB-D RTAB-Map profile requires scan_source:=none")
35:     if mode == "localization":
36:         if backend == "cartographer_2d":
37:             raise ContractError("Cartographer .pbstream pure-localization flow not tested")
38:         filename = database_path if backend == "rtabmap" else posegraph_file
39:         if not filename or not Path(filename).expanduser().is_file():
40:             raise ContractError("Localization needs existing native map asset: " + filename)
41:     return source
42: 
43: 
44: def validate_hardware_contract(path, backend, source):
45:     file_path = Path(path).expanduser()
46:     if not file_path.is_file():
47:         raise ContractError("Hardware verification JSON missing: " + str(file_path))
48:     try:
49:         data = json.loads(file_path.read_text(encoding="utf8"))
50:     except (OSError, ValueError) as exc:
51:         raise ContractError("Invalid hardware verification JSON: " + str(exc)) from exc
52:     required = ["wheel_odom_verified", "footprint_verified", "cmd_vel_verified"]
53:     if backend == "rtabmap" or source == "depth":
54:         required.append("camera_extrinsic_verified")
55:     else:
56:         required.append("lidar_extrinsic_verified")
57:     missing = [k for k in required if data.get(k) is not True]
58:     if data.get("odom_frame") != "odom" or data.get("base_frame") != "base_link":
59:         missing.append("frame convention odom/base_link")
60:     if missing:
61:         raise ContractError("Navigation BLOCKED: " + ", ".join(missing))
62:     return True