from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    trimaran_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('trimaran_description'),
                'launch',
                'display.launch.py'
            )
        )
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xsens_mti_ros2_driver'),
                'launch',
                'xsens_mti_node.launch.py'
            )
        )
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'base_link']
    )

    return LaunchDescription([
        xsens_launch,
        static_tf,
        trimaran_launch,
    ])
