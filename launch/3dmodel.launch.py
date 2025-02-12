from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os
import xacro

def generate_launch_description():
    package_name = "robot_package"
    pkg_share = get_package_share_directory(package_name)
    # Get the path to the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', '3dmodel.xacro')
    
    # Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
    # Start robot state publisher node
    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
        ),

    #Start joint state publisher node
    Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description]
        ),
])
