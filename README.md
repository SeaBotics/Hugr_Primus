# 🧭 Hugr_Primus – SeaBotics ROS 2 GUI

ROS 2 Humble GUI-system for Hugr-Primus utviklet av SeaBotics.

## Innhold
- HugrStatusPanel (RViz Panel)
- HugrCompassPanel (Eget RViz Panel)
- HugrGyroDisplay (3D RViz Display)
- hugr_fake_status (Test-node)

## Miljø
- Ubuntu 22.04
- ROS 2 Humble
- colcon
- RViz 2
- Jetson Orin Nano / WSL

## Struktur
ros2_ws/
└── src/
    └── Hugr_Primus/
        ├── hugr_rviz_panel/
        └── hugr_fake_status/

## RViz Plugins

### HugrStatusPanel
Viser:
- Batteri %
- Batteri temperatur
- Fart
- Retning (Heading)
- Posisjon (x,y,z)
- Yaw / Pitch / Roll
- Modus

### HugrCompassPanel
- Leser `/imu/data`
- 0° = North
- 90° = East
- 180° = South
- 270° = West
- Oppdateres kontinuerlig fra IMU yaw

### HugrGyroDisplay
3D display med:
- Yaw ring
- Pitch ring
- Roll ring

## Topics
/imu/data – sensor_msgs/Imu  
/cmd_vel – geometry_msgs/Twist  
/battery_percent – std_msgs/Float32  
/battery_temp – std_msgs/Float32  
/position – geometry_msgs/PointStamped  
/mode – std_msgs/String  

## Build
cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  
colcon build --symlink-install  
source install/setup.bash  

## Start RViz
rviz2  

Panels:
- hugr_rviz_panel/HugrStatusPanel
- hugr_rviz_panel/HugrCompassPanel

Displays:
- hugr_rviz_panel/HugrGyroDisplay

## Branch
gui_Panel

## Git Workflow
git pull  
git checkout gui_Panel  
git add .  
git commit -m "Message"  
git push  

Utviklet av SeaBotics 🚀
