from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Bygg robot_description med xacro (sim:=true)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('hugr_description'), 'urdf', 'asv.urdf.xacro'
    ])
    robot_description = ParameterValue(
        Command(['xacro', xacro_file, 'sim:=true']),
        value_type=str
    )

    # Start Gazebo (server + GUI)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Robot State Publisher (gir /tf og holder robot_description som PARAMETER)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}]
    )

    # Spawn modellen i Gazebo – LES fra PARAMETER i stedet for topic
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'hugr_asv',
            '-param', 'robot_description',   # <--- viktig endring
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # ros2_control-node + spawners (samme som før, men start ETTER Gazebo + spawn)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time},
                    # peker til din ros2_control YAML (i install/share/... etter build)
                    PathJoinSubstitution([
                        FindPackageShare('hugr_description'),
                        'ros2_control', 'hugr_ros2_control.yaml'
                    ])],
        output='both'
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    surge_front = Node(package='controller_manager', executable='spawner',
                       arguments=['thruster_surge_front_controller'], output='screen')
    surge_rear  = Node(package='controller_manager', executable='spawner',
                       arguments=['thruster_surge_rear_controller'], output='screen')
    sway_port   = Node(package='controller_manager', executable='spawner',
                       arguments=['thruster_sway_port_controller'], output='screen')
    sway_star   = Node(package='controller_manager', executable='spawner',
                       arguments=['thruster_sway_starboard_controller'], output='screen')

    # Vent litt så Gazebo rekker å komme opp før vi spawner og starter controllere
    delayed = TimerAction(period=5.0, actions=[
        spawn, jsb, surge_front, surge_rear, sway_port, sway_star
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        rsp,
        ros2_control_node,
        delayed
    ])
