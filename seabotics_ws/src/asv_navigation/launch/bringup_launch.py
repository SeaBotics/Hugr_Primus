import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Finn stien til pakken din
    nav_dir = get_package_share_directory('asv_navigation')
    
    # Finn stien til den offisielle nav2_bringup-pakken
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Pek på filene dine
    params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    bt_xml_file = os.path.join(nav_dir, 'behavior_trees', 'navigate_to_pose.xml')

    # Inkluder standard-launch-filen til Nav2, men bruk DINE parametere
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'default_bt_xml_filename': bt_xml_file,
            'use_sim_time': 'False'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    return ld