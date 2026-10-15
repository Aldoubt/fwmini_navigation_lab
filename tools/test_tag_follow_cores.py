"""Compile actual detector/follower cores without ROS; validates fixture and syntax."""
import argparse
import ast
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cxx', default='g++')
    args = parser.parse_args()
    for package_name in ('fwmini_perception', 'apriltag_detector_adapter', 'tag_follow_controller'):
        package = ROOT / 'src' / package_name
        assert ET.parse(package / 'package.xml').getroot().findtext('name') == package_name
        for path in package.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        if package_name == 'fwmini_perception':
            continue
        with tempfile.TemporaryDirectory(prefix='fwmini_tag_core_') as tmp:
            binary = Path(tmp) / 'test_core.exe'
            subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                            '-I', str(package / 'include'), str(package / 'test/test_core.cpp'),
                            '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    with tempfile.TemporaryDirectory(prefix='fwmini_tag_chain_') as tmp:
        binary = Path(tmp) / 'test_chain.exe'
        includes = []
        for name in ('apriltag_detector_adapter', 'tag_follow_controller', 'fwmini_chassis_adapter'):
            includes += ['-I', str(ROOT / 'src' / name / 'include')]
        subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                        *includes, str(ROOT / 'tools/test_tag_chain.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    ast.parse((ROOT / 'src/fwmini_bringup/launch/tag_follow.launch.py').read_text(encoding='utf-8'))
    print('PASS: quality/follow C++ core compilation and execution, XML and launch/test syntax')
    print('Real ROS image/detection/TF/CtrlCmd integration is separate: test_pipeline.py')


if __name__ == '__main__':
    main()
