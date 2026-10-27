"""Run the workbench hardware-free Humble suite; never starts the CAN driver."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    suite = Path(__file__).resolve().parents[4]/'agt_field_test_workbench/tests/humble_fwmini_control.py'
    runpy.run_path(str(suite), run_name='__main__')
