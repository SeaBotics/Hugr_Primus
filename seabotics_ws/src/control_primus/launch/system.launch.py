import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Finn veien til YAML-fila vår i install-mappa
    config = os.path.join(
        get_package_share_directory('control_primus'),
        'config',
        'control_params.yaml'
    )

    # 2. Definer nodene som skal startes
    velocity_node = Node(
        package='control_primus',
        executable='velocity_node',
        name='velocity_node',
        parameters=[config], # Her kobler vi på YAML-fila!
        output='screen'
    )

    allocation_node = Node(
        package='control_primus',
        executable='allocation_node',
        name='allocation_node',
        parameters=[config],
        output='screen'
    )

    arduino_bridge = Node(
        package='control_primus',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([
        velocity_node,
        allocation_node,
        arduino_bridge
    ])
