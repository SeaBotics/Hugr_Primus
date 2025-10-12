from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    sim = LaunchConfiguration('sim')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('hugr_description'), 'urdf', 'asv.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            xacro_file, ' sim:=', sim
        ])
    }

    rviz_config = PathJoinSubstitution([
        FindPackageShare('hugr_bringup'), 'rviz', 'hugr.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[robot_description],
            output='screen'
        ),
    ])
