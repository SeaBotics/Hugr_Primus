from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy Node (Driver for kontroller)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # ASV Master Node
        Node(
            package='asv_control',
            executable='asv_master_node',
            name='asv_master',
            output='screen'
        )
    ])