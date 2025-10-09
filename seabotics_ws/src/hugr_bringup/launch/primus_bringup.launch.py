# Purpose: One-button launcher for the whole ROS 2 stack in seabotics_ws
# Supports sim/hw modes, NAV2, manual control, allocator/control, and RViz.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------------------- Arguments ----------------------
    use_sim = LaunchConfiguration('use_sim')                 # 'true' -> Gazebo Sim stack
    use_nav2 = LaunchConfiguration('use_nav2')               # bring up Nav2
    use_manual = LaunchConfiguration('use_manual')           # bring up joystick/manual control
    use_rviz = LaunchConfiguration('use_rviz')               # bring up RViz
    namespace = LaunchConfiguration('namespace')             # e.g., 'hugr'

    world = LaunchConfiguration('world')                     # path to .sdf world (sim only)
    urdf_pkg = LaunchConfiguration('urdf_pkg')               # default: hugr_description
    urdf_file = LaunchConfiguration('urdf_file')             # default: urdf/hugr.urdf.xacro

    nav2_params = LaunchConfiguration('nav2_params')         # path to nav2 params
    ekf_params = LaunchConfiguration('ekf_params')           # path to robot_localization params

    # ---------------------- Defaults -----------------------
    hugr_description_share = get_package_share_directory('hugr_description')
    hugr_bringup_share = get_package_share_directory('hugr_bringup')

    declare_args = [
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_manual', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='hugr'),
        DeclareLaunchArgument('world', default_value=os.path.join(hugr_bringup_share, 'worlds', 'ocean.sdf')),
        DeclareLaunchArgument('urdf_pkg', default_value='hugr_description'),
        DeclareLaunchArgument('urdf_file', default_value=os.path.join(hugr_description_share, 'urdf', 'hugr.urdf.xacro')),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(hugr_bringup_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('ekf_params', default_value=os.path.join(hugr_bringup_share, 'config', 'robot_localization.yaml')),
    ]

    # ---------------- robot_state_publisher (URDF) ---------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim')}],
        arguments=['-d', urdf_file],
        output='screen')

    # ---------------- robot_localization (EKF) -------------
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        namespace=namespace, output='screen',
        parameters=[ekf_params])

    # ---------------- Thruster allocator/control ------------
    # If you implemented hugr_control allocator_node
    allocator = Node(
        package='hugr_control', executable='allocator_node', name='allocator',
        namespace=namespace, output='screen',
        parameters=[os.path.join(hugr_bringup_share, 'config', 'control.yaml')],
        condition=IfCondition('true'))  # always on; adjust if needed

    # ---------------- Thruster bridge (sim) ----------------
    thruster_bridge = Node(
        package='hugr_bringup', executable='thruster_bridge.py', name='thruster_bridge',
        namespace=namespace, output='screen',
        condition=IfCondition(use_sim))

    # ---------------- Gazebo Sim (optional) ----------------
    # Start gz sim with world; you may replace with your preferred launch if you use ros_gz_sim.
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', '-s', '--headless-rendering', world],
        output='screen', condition=IfCondition(use_sim))

    # ---------------- NAV2 bringup (optional) --------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim,
            'params_file': nav2_params,
            'autostart': 'true',
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # ---------------- Manual control (optional) ------------
    manual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hugr_bringup_share, 'launch', 'manual_joy.launch.py')
        ),
        launch_arguments={'namespace': namespace}.items(),
        condition=IfCondition(use_manual)
    )

    # ---------------- RViz (optional) ----------------------
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(hugr_bringup_share, 'config', 'nav2_default_view.rviz')],
        condition=IfCondition(use_rviz)
    )

    # Force global use_sim_time for nodes in same process
    use_sim_param = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim'))

    return LaunchDescription([
        *declare_args,
        use_sim_param,
        gz,
        robot_state_publisher,
        ekf,
        allocator,
        thruster_bridge,
        nav2_bringup,
        manual,
        rviz,
    ])
