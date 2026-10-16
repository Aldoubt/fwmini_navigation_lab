1: """Dependency-free offline tests of the launch eligibility rules."""
2: import json
3: from pathlib import Path
4: import sys
5: from tempfile import TemporaryDirectory
6: import unittest
7: 
8: ROOT = Path(__file__).resolve().parents[1]
9: PKG = ROOT / "fwmini_multi_nav_bringup"
10: if not PKG.is_dir():
11:     PKG = ROOT / "src" / "fwmini_multi_nav_bringup"
12: sys.path.insert(0, str(PKG))
13: from fwmini_multi_nav_bringup.contracts import (  # noqa
14:     ContractError, resolve_scan_source, validate_hardware_contract,
15:     validate_selection,
16: )
17: 
18: 
19: class SelectionTests(unittest.TestCase):
20:     def test_auto_sources(self):
21:         self.assertEqual(resolve_scan_source("rtabmap", "auto"), "none")
22:         self.assertEqual(resolve_scan_source("slam_toolbox", "auto"), "depth")
23:         self.assertEqual(resolve_scan_source("cartographer_2d", "auto"), "depth")
24: 
25:     def test_approved_mapping_profiles(self):
26:         for backend, scan in (
27:             ("rtabmap", "none"),
28:             ("slam_toolbox", "depth"),
29:             ("cartographer_2d", "cloud"),
30:         ):
31:             self.assertEqual(validate_selection(backend, "mapping", scan), scan)
32: 
33:     def test_rejected_profiles(self):
34:         cases = [
35:             ("unknown", "mapping", "none"),
36:             ("rtabmap", "mapping", "cloud"),
37:             ("slam_toolbox", "mapping", "none"),
38:             ("cartographer_2d", "mapping", "none"),
39:             ("fastlio2", "mapping", "auto"),
40:             ("cartographer_2d", "localization", "depth"),
41:             ("rtabmap", "wrong", "none"),
42:         ]
43:         for backend, mode, source in cases:
44:             with self.subTest(backend=backend, mode=mode):
45:                 with self.assertRaises(ContractError):
46:                     validate_selection(backend, mode, source)
47: 
48:     def test_native_map_required_for_localization(self):
49:         with self.assertRaises(ContractError):
50:             validate_selection("rtabmap", "localization", "none", "/missing/db")
51:         with self.assertRaises(ContractError):
52:             validate_selection("slam_toolbox", "localization", "depth", posegraph_file="/missing/map")
53:         with TemporaryDirectory() as tmp:
54:             native = Path(tmp) / "map.db"
55:             native.write_bytes(b"dummy")
56:             self.assertEqual(
57:                 validate_selection("rtabmap", "localization", "none", database_path=str(native)),
58:                 "none",
59:             )
60: 
61: 
62: class NavGateTests(unittest.TestCase):
63:     def test_unverified_template_rejected(self):
64:         contract = PKG / "config/hardware_contract.json"
65:         for backend, source in (
66:             ("rtabmap", "none"), ("slam_toolbox", "depth"),
67:             ("cartographer_2d", "cloud"),
68:         ):
69:             with self.assertRaises(ContractError):
70:                 validate_hardware_contract(contract, backend, source)
71: 
72:     def test_verified_camera_does_not_approve_lidar(self):
73:         with TemporaryDirectory() as tmp:
74:             contract = Path(tmp) / "hardware.json"
75:             data = json.loads((PKG / "config/hardware_contract.json").read_text())
76:             for key in (
77:                 "wheel_odom_verified", "footprint_verified",
78:                 "cmd_vel_verified", "camera_extrinsic_verified",
79:             ):
80:                 data[key] = True
81:             contract.write_text(json.dumps(data))
82:             self.assertTrue(validate_hardware_contract(contract, "rtabmap", "none"))
83:             with self.assertRaises(ContractError):
84:                 validate_hardware_contract(contract, "cartographer_2d", "cloud")
85: 
86:     def test_missing_contract_rejected(self):
87:         with self.assertRaises(ContractError):
88:             validate_hardware_contract("/missing/hardware.json", "rtabmap", "none")
89: 
90: 
91: class SourceContractTests(unittest.TestCase):
92:     def test_cartographer_single_tf_owner(self):
93:         lua = (PKG / "config/cartographer/fwmini_2d.lua").read_text()
94:         for expected in (
95:             'published_frame = "odom"', "provide_odom_frame = false",
96:             "use_odometry = true",
97:         ):
98:             self.assertIn(expected, lua)
99: 
100:     def test_rtabmap_does_not_launch_second_odom(self):
101:         source = (PKG / "launch/bringup.launch.py").read_text()
102:         self.assertNotIn('executable="rgbd_odometry"', source)
103:         self.assertIn('("navigation", "false"', source)
104:         self.assertIn('("odom_topic", "/wheel/odom"', source)
105:         self.assertIn('("rgb_topic", "/fwmini/camera/color/image_raw"', source)
106: 
107:     def test_config_artifacts_exist(self):
108:         for path in (
109:             "launch/bringup.launch.py", "config/slam_toolbox/mapping.yaml",
110:             "config/slam_toolbox/localization.yaml", "config/cartographer/fwmini_2d.lua",
111:         ):
112:             self.assertTrue((PKG / path).is_file(), path)
113: 
114: 
115: if __name__ == "__main__":
116:     unittest.main()