import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ========================================================================
    # FASE 0: ROBOTENS FYSISKE BESKRIVELSE (URDF / TF)
    # ========================================================================
    urdf_file_path = os.path.join(get_package_share_directory('primus_bringup'), 'urdf', 'primus.urdf')
    
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ========================================================================
    # 1. DEFINERER STIER TIL PAKKENE
    # ========================================================================
    io_dir           = get_package_share_directory('io_primus')
    teleop_dir       = get_package_share_directory('teleoperation')
    mode_dir         = get_package_share_directory('mode_manager')
    control_dir      = get_package_share_directory('control_primus')
    nav_dir          = get_package_share_directory('navigation_primus')
    guidance_dir     = get_package_share_directory('guidance_primus')
    mission_dir      = get_package_share_directory('mission_control')

    # ========================================================================
    # FASE 1: MASKINVARE OG SANSING (Input/Output)
    # ========================================================================
    
    # Arduino Bridge: Håndterer seriell kommunikasjon og PWM
    io_node = Node(
        package='io_primus',
        executable='arduino_bridge',
        name='arduino_bridge_node',
        output='screen'
    )

    # Teleoperation: Leser Xbox-kontroller og genererer Wrench (Newton)
    teleop_node = Node(
        package='teleoperation',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen'
    )

    # ========================================================================
    # FASE 2: SIKKERHET OG MODUS (Dørvakta)
    # ========================================================================
    
    # Mode Manager: Starter alltid i Modus 0 (Killswitch)
    mode_manager_node = Node(
        package='mode_manager',
        executable='mode_manager_node',
        name='mode_manager_node',
        output='screen'
    )

    # Mission Control: Overvåker modus og avbryter Nav2 ved behov
    mission_control_node = Node(
        package='mission_control',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen'
    )

    # ========================================================================
    # FASE 3: KONTROLLSYSTEM (Musklene)
    # ========================================================================
    
    # Velocity Controller: PID-regulator for fart og retning
    velocity_controller_node = Node(
        package='control_primus',
        executable='velocity_controller_node',
        name='velocity_node',
        output='screen'
    )

    # Thrust Allocation: Oversetter krefter til de 4 individuelle motorene
    thrust_allocation_node = Node(
        package='control_primus',
        executable='thrust_allocation_node',
        name='allocation_node',
        output='screen'
    )

    # ========================================================================
    # FASE 4: NAVIGASJON OG SENSORFUSJON (Hvor er vi?)
    # Starter etter 2 sekunder for å la maskinvaren initialisere seg
    # ========================================================================
    navigation_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_dir, 'launch', 'navigation.launch.py')
                )
            )
        ]
    )

    # ========================================================================
    # FASE 5: GUIDANCE OG AUTOPILOT (Hvor skal vi?)
    # Venter 4 sekunder slik at EKF-en har rukket å publisere TF-transformene
    # ========================================================================
    guidance_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(guidance_dir, 'launch', 'guidance.launch.py')
                )
            )
        ]
    )

    # ========================================================================
    # OPPSTART AV HELE SYSTEMET
    # ========================================================================
    return LaunchDescription([
        # Grunnleggende noder
        robot_state_publisher_node,
        io_node,
        teleop_node,
        mode_manager_node,
        mission_control_node,
        velocity_controller_node,
        thrust_allocation_node,
        
        # Forsinkede oppstarter for stabilitet
        navigation_launch,
        guidance_launch
    ])
