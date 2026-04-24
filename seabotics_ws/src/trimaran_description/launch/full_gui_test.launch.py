import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('trimaran_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'trimaran_description.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    map_yaml = os.path.expanduser('~/kartverket_rviz/map.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    odom_to_base_link_test = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_test',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    mode_bridge = Node(
        package='trimaran_description',
        executable='mode_bridge.py',
        name='mode_bridge',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager,
        world_to_map,
        map_to_odom,
        odom_to_base_link_test,
        robot_state_publisher,
        mode_bridge,
        rviz2
    ])
