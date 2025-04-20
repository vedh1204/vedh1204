from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'pointcloud_bot',
                '-file', '/home/vedh/vedh_ros/src/vedh_gazebo/urdf/pointcloud_bot.urdf',
                '-robot_namespace', 'pointcloud_bot'
            ],
            output='screen'
        ),
    ])
