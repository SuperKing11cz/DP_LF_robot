from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    # Cesta k robot_package balíčku a world souboru
    package_name = 'robot_package'
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    # Cesty k důležitým souborům
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    rsp_launch_file = os.path.join(pkg_share, 'launch', 'car.launch.py')


    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Použít simulovaný čas, pokud je true'
        ),

        # Aby Gazebo našel modely
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.expanduser('~/dp2025_ws/src/')
        ),

        # Deklarace argumentu pro world soubor
        DeclareLaunchArgument(
            name='world',
            default_value=world_file,
            description='Path to the world file to load'
        ),
        
        # Spuštění Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world')],
            output='screen'
        ),

        # Spuštění rsp_launch_file.py (robot_state_publisher a joint_state_publisher)
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
