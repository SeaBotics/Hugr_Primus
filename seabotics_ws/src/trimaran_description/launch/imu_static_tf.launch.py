from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'base_link']
        ),
    ])
