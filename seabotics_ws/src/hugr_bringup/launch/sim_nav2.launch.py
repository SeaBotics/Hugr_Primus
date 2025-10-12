from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('sim', default_value='true')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('hugr_description'), 'urdf', 'asv.urdf.xacro'
    ])
    ros2_control_yaml = PathJoinSubstitution([
        FindPackageShare('hugr_description'), 'ros2_control', 'hugr_ros2_control.yaml'
    ])

    # xacro -> robot_description
    robot_description_cmd = Command(['xacro ', urdf_path, ' sim:=', LaunchConfiguration('sim')])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_cmd, 'use_sim_time': True}],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description_cmd}, ros2_control_yaml],
    )

    # ÉN spawner, sekvensielt, med lang timeout
    spawn_all = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'thruster_surge_front_controller',
            'thruster_surge_rear_controller',
            'thruster_sway_port_controller',
            'thruster_sway_starboard_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '45',
        ],
    )

    return LaunchDescription([
        sim_arg,
        rsp,
        control_node,
        # vent litt så pluginloader er helt ferdig før vi spawner
        TimerAction(period=8.0, actions=[spawn_all]),
    ])
