from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('trimaran_description')

    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_path, 'config', 'navsat.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_config]
    )

    return LaunchDescription([
        ekf_node,
        navsat_node
    ])