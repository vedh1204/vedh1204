from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_rect_raw'),
                ('rgb/camera_info', '/camera/color/camera_info')
            ]
        )
    ])
