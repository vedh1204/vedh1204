from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_file_name = 'custom_world.world'
    world_path = os.path.join(
        FindPackageShare('vedh_gazebo').find('vedh_gazebo'),
        'worlds',
        world_file_name
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'world': world_path}.items()
        )
    ])


