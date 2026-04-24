from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Finn stien til konfigurasjonsfilen vår
    pkg_path = get_package_share_directory('navigation_primus')
    config_file = os.path.join(pkg_path, 'config', 'xsens_ekf.yaml')

    return LaunchDescription([
        # 1. Lokal EKF (odom -> base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            parameters=[config_file],
            remappings=[('odometry/filtered', 'odometry/filtered_local')]
        ),

        # 2. Global EKF (map -> odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            output='screen',
            parameters=[config_file],
            remappings=[('odometry/filtered', 'odometry/filtered_global')]
        ),

        # 3. Navsat Transform (GPS -> Meter)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('imu', 'imu/data'),
                ('gps/fix', 'gnss'),
                ('odometry/filtered', 'odometry/filtered_global')
            ]
        )
    ])
