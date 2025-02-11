from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    package_name = 'robot_package'
    pkg_share = get_package_share_directory(package_name)

    # Cesty k důležitým souborům
    xacro_file = os.path.join(pkg_share, 'urdf', 'lf_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    robot_description = {'robot_description': xacro.process_file(xacro_file)}

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Použít simulovaný čas, pokud je true'
        ),

        # Spuštění Gazebo s konkrétním světem
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'gz_args': '-r ' + world_file}.items()
        ),

        # Publikování popisu robota
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),

        # Vložení robota do Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot', '-file', urdf_file],
            output='screen'
        )
    ])
