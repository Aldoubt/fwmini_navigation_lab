"""Dependency-free offline tests of the launch eligibility rules."""
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "fwmini_multi_nav_bringup"
if not PKG.is_dir():
    PKG = ROOT / "src" / "fwmini_multi_nav_bringup"
sys.path.insert(0, str(PKG))
from fwmini_multi_nav_bringup.contracts import (  # noqa
    ContractError, resolve_scan_source, validate_hardware_contract,
    validate_selection,
)


class SelectionTests(unittest.TestCase):
    def test_auto_sources(self):
        self.assertEqual(resolve_scan_source("rtabmap", "auto"), "none")
        self.assertEqual(resolve_scan_source("slam_toolbox", "auto"), "depth")
        self.assertEqual(resolve_scan_source("cartographer_2d", "auto"), "depth")

    def test_approved_mapping_profiles(self):
        for backend, scan in (
            ("rtabmap", "none"),
            ("slam_toolbox", "depth"),
            ("cartographer_2d", "cloud"),
        ):
            self.assertEqual(validate_selection(backend, "mapping", scan), scan)

    def test_rejected_profiles(self):
        cases = [
            ("unknown", "mapping", "none"),
            ("rtabmap", "mapping", "cloud"),
            ("slam_toolbox", "mapping", "none"),
            ("cartographer_2d", "mapping", "none"),
            ("fastlio2", "mapping", "auto"),
            ("cartographer_2d", "localization", "depth"),
            ("rtabmap", "wrong", "none"),
        ]
        for backend, mode, source in cases:
            with self.subTest(backend=backend, mode=mode):
                with self.assertRaises(ContractError):
                    validate_selection(backend, mode, source)

    def test_native_map_required_for_localization(self):
        with self.assertRaises(ContractError):
            validate_selection("rtabmap", "localization", "none", "/missing/db")
        with self.assertRaises(ContractError):
            validate_selection("slam_toolbox", "localization", "depth", posegraph_file="/missing/map")
        with TemporaryDirectory() as tmp:
            native = Path(tmp) / "map.db"
            native.write_bytes(b"dummy")
            self.assertEqual(
                validate_selection("rtabmap", "localization", "none", database_path=str(native)),
                "none",
            )


class NavGateTests(unittest.TestCase):
    def test_unverified_template_rejected(self):
        contract = PKG / "config/hardware_contract.json"
        for backend, source in (
            ("rtabmap", "none"), ("slam_toolbox", "depth"),
            ("cartographer_2d", "cloud"),
        ):
            with self.assertRaises(ContractError):
                validate_hardware_contract(contract, backend, source)

    def test_verified_camera_does_not_approve_lidar(self):
        with TemporaryDirectory() as tmp:
            contract = Path(tmp) / "hardware.json"
            data = json.loads((PKG / "config/hardware_contract.json").read_text())
            for key in (
                "wheel_odom_verified", "footprint_verified",
                "cmd_vel_verified", "camera_extrinsic_verified",
            ):
                data[key] = True
            contract.write_text(json.dumps(data))
            self.assertTrue(validate_hardware_contract(contract, "rtabmap", "none"))
            with self.assertRaises(ContractError):
                validate_hardware_contract(contract, "cartographer_2d", "cloud")

    def test_missing_contract_rejected(self):
        with self.assertRaises(ContractError):
            validate_hardware_contract("/missing/hardware.json", "rtabmap", "none")


class SourceContractTests(unittest.TestCase):
    def test_cartographer_single_tf_owner(self):
        lua = (PKG / "config/cartographer/fwmini_2d.lua").read_text()
        for expected in (
            'published_frame = "odom"', "provide_odom_frame = false",
            "use_odometry = true",
        ):
            self.assertIn(expected, lua)

    def test_rtabmap_does_not_launch_second_odom(self):
        source = (PKG / "launch/bringup.launch.py").read_text()
        self.assertNotIn('executable="rgbd_odometry"', source)
        self.assertIn('("navigation", "false"', source)
        self.assertIn('("odom_topic", "/wheel/odom"', source)
        self.assertIn('("rgb_topic", "/fwmini/camera/color/image_raw"', source)

    def test_config_artifacts_exist(self):
        for path in (
            "launch/bringup.launch.py", "config/slam_toolbox/mapping.yaml",
            "config/slam_toolbox/localization.yaml", "config/cartographer/fwmini_2d.lua",
        ):
            self.assertTrue((PKG / path).is_file(), path)


if __name__ == "__main__":
    unittest.main()
