from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    package_name = 'robot_package'
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    # Cesty k důležitým souborům
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    rsp_launch_file = os.path.join(pkg_share, 'launch', '3dmodel.launch.py')


    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Použít simulovaný čas, pokud je true'
        ),

        DeclareLaunchArgument(
            name='world',
            default_value=world_file,
            description='Path to the world file to load'
        ),
        
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world')],
            output='screen'
        ),

        # Include the rsp.launch.py file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_file)
        ),

        # Vložení robota do Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description'],
            output='screen',
        )
    ])
