from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_nav2   = LaunchConfiguration('use_nav2')
    use_manual = LaunchConfiguration('use_manual')
    use_rviz   = LaunchConfiguration('use_rviz')
    namespace  = LaunchConfiguration('namespace')
    world      = LaunchConfiguration('world')
    urdf_file  = LaunchConfiguration('urdf_file')
    nav2_params = LaunchConfiguration('nav2_params')
    ekf_params  = LaunchConfiguration('ekf_params')

    desc_share    = get_package_share_directory('hugr_description')
    bringup_share = get_package_share_directory('hugr_bringup')

    declare = [
        DeclareLaunchArgument('use_nav2',  default_value='true'),
        DeclareLaunchArgument('use_manual',default_value='true'),
        DeclareLaunchArgument('use_rviz',  default_value='false'),
        DeclareLaunchArgument('namespace', default_value='hugr'),
        DeclareLaunchArgument('world',     default_value=os.path.join(bringup_share, 'worlds', 'ocean.sdf')),
        DeclareLaunchArgument('urdf_file', default_value=os.path.join(desc_share, 'urdf', 'hugr.urdf.xacro')),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('ekf_params',  default_value=os.path.join(bringup_share, 'config', 'robot_localization.yaml')),
    ]

    # Start gz sim (Fortress)
    gz = ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '3', '-s', '--headless-rendering', world],
                        output='screen')

    # Publiser URDF (xacro) som TF
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace=namespace, parameters=[{'use_sim_time': True}],
               arguments=['-d', urdf_file], output='screen')

    # Spawn modellen i gz-sim
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-file', urdf_file, '-name', 'hugr'], output='screen')

    # Minste bro (utvid senere med sensorer du trenger)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=[
                      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                  ])

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               parameters=[ekf_params], output='screen', namespace=namespace)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time':'True','params_file':nav2_params,'autostart':'true','namespace':namespace}.items(),
        condition=IfCondition(use_nav2)
    )

    manual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'manual_joy.launch.py')),
        launch_arguments={'namespace': namespace}.items(),
        condition=IfCondition(use_manual)
    )

    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', os.path.join(bringup_share, 'config', 'nav2_default_view.rviz')],
                condition=IfCondition(use_rviz))

    return LaunchDescription(declare + [
        SetParameter(name='use_sim_time', value=True),
        gz, rsp, spawn, bridge, ekf, nav2, manual, rviz
    ])
