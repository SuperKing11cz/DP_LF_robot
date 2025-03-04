from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robot_package'
    pkg_share = get_package_share_directory(pkg_name)
    config_file_path = os.path.join(pkg_share, 'config', 'config_sensors.yaml')

    bridge_name = LaunchConfiguration('sensors_bridge', default='default_bridge')
    config_file = LaunchConfiguration('config_sensors', default=config_file_path)

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'sensors_bridge', default_value='default_bridge', description='Nazev ros_gz_bridge nodu'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_sensors', default_value=config_file_path, description='YAML config file'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        declare_bridge_name_cmd,
        declare_config_file_cmd,
        RosGzBridge(
            bridge_name=bridge_name,
            config_file=config_file,
        ),
    ])

    return ld
