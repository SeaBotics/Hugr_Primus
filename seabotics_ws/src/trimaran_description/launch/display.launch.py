import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Finn stien til pakken din i install-mappen
    pkg_share = get_package_share_directory('trimaran_description')
    
    # PEK PÅ RIKTIG FIL: urdf-mappen og filnavnet med .urdf til slutt
    default_model_path = os.path.join(pkg_share, 'urdf', 'trimaran_description.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # Robot State Publisher (bruker xacro for å lese urdf-en)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])