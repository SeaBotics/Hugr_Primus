import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Finner mappene vi trenger
    guidance_dir = get_package_share_directory('guidance_primus')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Peker på YAML-filen vi nettopp laget
    params_file = os.path.join(guidance_dir, 'config', 'nav2_params.yaml')

    # Vi inkluderer Nav2 sin offisielle launch-fil, men tvinger den til å bruke VÅR oppskrift
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        bringup_cmd
    ])
