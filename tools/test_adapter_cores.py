"""Compile and execute real adapter cores without ROS or robot hardware."""
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
    for name in ('fwmini_chassis_adapter', 'fwmini_odom_adapter'):
        package = ROOT / 'src' / name
        assert ET.parse(package / 'package.xml').getroot().findtext('name') == name
        for path in package.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        with tempfile.TemporaryDirectory(prefix='fwmini_core_') as temporary:
            binary = Path(temporary) / 'test_core.exe'
            subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-pedantic', '-I', str(package / 'include'),
                            str(package / 'test/test_core.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    print('PASS: both C++ cores compiled/executed; Python syntax and package XML checked')
    print('ROS nodes / DDS / TF graph are NOT validated by this command.')


if __name__ == '__main__':
    main()
