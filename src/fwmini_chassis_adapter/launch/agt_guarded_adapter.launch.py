"""FW-mini adapter taking commands only from the AGT guarded topic.

This launch does not start CAN, Nav2 or FAST-LIO2 and does not enable motion.
The adapter's existing enable service remains disabled until explicitly called.
"""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('fwmini_chassis_adapter')) /
        'config/agt_guarded_cmd_adapter.yaml')
    # Multiple same-named ROS message packages exist on this computer:
    # ros2_ws/install/yhs_can_interfaces (older, incompatible CtrlCmd ABI)
    # and FW-mini's private yhs_can_interfaces (4-field CtrlCmd).
    # Force the adapter subprocess to load its own linked FW-mini variant
    # BEFORE inherited ros2_ws libraries. Without this, DDS reports the same
    # type name but serializes an incompatible payload and the CAN VCU sees
    # no meaningful command.
    private_prefix = Path(get_package_prefix('fwmini_chassis_adapter')).parent
    private_yhs = private_prefix / 'yhs_can_interfaces'
    private_schema = private_yhs / 'share/yhs_can_interfaces/msg/CtrlCmd.msg'
    vendor_schema = (Path.home() /
                     'fwmini_ws/project/install/vendor/yhs_can_interfaces/share/'
                     'yhs_can_interfaces/msg/CtrlCmd.msg')
    if not private_schema.is_file() or not vendor_schema.is_file():
        raise RuntimeError('FW-mini CtrlCmd schema missing from adapter/vendor overlays')
    if private_schema.read_bytes() != vendor_schema.read_bytes():
        raise RuntimeError('FW-mini CtrlCmd interface ABI differs from vendor driver')
    private_lib = private_yhs / 'lib'
    if not private_lib.is_dir():
        raise RuntimeError(f'FW-mini interface library directory missing: {private_lib}')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('initial_motion_enabled', default_value='false'),
        Node(
            package='fwmini_chassis_adapter',
            executable='fwmini_chassis_adapter_node',
            name='fwmini_chassis_adapter',
            output='screen',
            # Override inherited LD_LIBRARY_PATH for this subprocess only.
            additional_env={'LD_LIBRARY_PATH': [
                str(private_lib), ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]},
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'initial_motion_enabled': ParameterValue(LaunchConfiguration('initial_motion_enabled'), value_type=bool)}]),
    ])
