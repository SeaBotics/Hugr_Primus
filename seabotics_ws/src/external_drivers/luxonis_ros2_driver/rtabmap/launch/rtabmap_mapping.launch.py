from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=['rtabmap_params.yaml'],
            remappings=[
                    ('rgb/image', '/oak/rgb/image_rect'),
                    ('rgb/camera_info', '/oak/rgb/camera_info'),
                    ('depth/image', '/oak/stereo/image_raw')
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen'
        )
    ])