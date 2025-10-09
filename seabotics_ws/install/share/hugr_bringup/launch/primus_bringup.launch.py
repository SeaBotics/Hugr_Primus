from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

world = os.path.join(get_package_share_directory('hugr_bringup'), 'worlds', 'ocean.world')
gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
    output='screen'
)

def generate_launch_description():
    desc_share = get_package_share_directory('hugr_description')
    bringup_share = get_package_share_directory('hugr_bringup')

    xacro_file = os.path.join(desc_share, 'urdf', 'hugr.urdf.xacro')
    world_file = os.path.join(bringup_share, 'worlds', 'ocean.sdf')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        ),

        # Start Gazebo Classic via systembinær
        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),

        # Spawn robot fra /robot_description
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', 'hugr_primus', '-topic', 'robot_description']
        ),

        # ros2_control controllere
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['thruster_controller'], output='screen'),
    ])
