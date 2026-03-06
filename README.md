# 🧭 Hugr Primus – SeaBotics ROS 2 GUI

GUI og visualiseringssystem for **Hugr-Primus** utviklet av **SeaBotics**.

Systemet er bygget på **ROS 2 Humble + RViz plugins** og brukes til å overvåke status fra sensorer på fartøyet.

GUI-en viser blant annet:

- Batteristatus
- Navigasjon og IMU-data
- Fart og posisjon
- Leak detection i skroget
- Kompass
- Gyro visualisering

Systemet kan også kjøres med **fake sensordata** for testing uten hardware.

---

# Systemoversikt

GUI består av flere **RViz plugins**.

| Plugin | Beskrivelse |
|------|------|
| HugrStatusPanel | Statuspanel med sensorinformasjon |
| HugrCompassPanel | Kompass basert på IMU yaw |
| HugrGyroDisplay | 3D gyroskop visualisering |
| HugrLeakPanel | Skrogvisualisering med lekkasjeindikasjon |

---

# HugrStatusPanel

Statuspanelet viser sanntidsdata fra ROS topics.

### Viste verdier

- Battery %
- Battery temperature
- Speed (m/s)
- Acceleration (|a|)
- Heading
- Position (x,y,z)
- Yaw
- Pitch
- Roll
- 5G Signal (dBm)
- Hull Temperature (°C)
- Mode (Kill Switch / Manual / Autonomous)

Mode vises med fargekode:

| Mode | Farge |
|-----|------|
| Kill Switch | Rød |
| Manual | Gul |
| Autonomous | Grønn |

---

# HugrLeakPanel

Leak-panelet viser en visualisering av skroget.

Skroget er delt i **2 seksjoner**.

| Farge | Betydning |
|-----|------|
| Hvit | Ingen lekkasje |
| Rød | Lekkasjedeteksjon |

Panelet abonnerer på topic:

/leak/levels

Type:

std_msgs/Float32MultiArray

Dataformat:

[front_section, rear_section]

Eksempel:

[0.0, 1.0]

Front = OK  
Rear = Leak

---

# ROS Topics

| Topic | Type | Beskrivelse |
|------|------|------|
| /imu/data | sensor_msgs/Imu | IMU orientasjon |
| /cmd_vel | geometry_msgs/Twist | Hastighet |
| /battery_percent | std_msgs/Float32 | Batteriprosent |
| /battery_temp | std_msgs/Float32 | Batteritemperatur |
| /position | geometry_msgs/PointStamped | Posisjon |
| /mode | std_msgs/String | Operasjonsmodus |
| /leak/levels | std_msgs/Float32MultiArray | Lekkasjenivå |
| /5g_signal | std_msgs/Float32 | Signalstyrke |
| /hull_temp | std_msgs/Float32 | Skrogtemperatur |

---

# Fake Sensor Nodes

For testing uten hardware finnes **fake nodes**.

Package:

hugr_fake_status

### Fake IMU / Status

Publiserer:

- IMU data
- battery
- position
- mode

Kjør:

ros2 run hugr_fake_status fake_status

---

### Fake Leak

Simulerer lekkasje i skroget.

Publiserer:

/leak/levels

Kjør:

ros2 run hugr_fake_status fake_leak

---

# Workspace Struktur

ros2_ws
└── src
    └── Hugr_Primus
        └── seabotics_ws
            └── src
                └── gui
                    ├── hugr_rviz_panel
                    └── hugr_fake_status

---

# Build

cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  

colcon build --symlink-install  

source install/setup.bash

---

# Start GUI

Start RViz:

rviz2

Legg til paneler:

Panels → Add New Panel

Velg:

hugr_rviz_panel/HugrStatusPanel  
hugr_rviz_panel/HugrCompassPanel  
hugr_rviz_panel/HugrLeakPanel  

Legg til display:

Displays → Add  

hugr_rviz_panel/HugrGyroDisplay

---

# Git Workflow

Arbeid skjer i branch:

gui_Panel

Standard workflow:

git pull  
git checkout gui_Panel  
git add .  
git commit -m "Update GUI"  
git push  

---

# Systemkrav

- Ubuntu 22.04
- ROS 2 Humble
- RViz2
- colcon
- C++17

Støttet hardware:

- Jetson Orin Nano
- Ubuntu PC
- WSL2

---

# SeaBotics

Prosjekt utviklet av **SeaBotics studentforening**.

Mål:

- bygge autonome maritime systemer
- ROS-basert kontrollarkitektur
- sensorintegrasjon
- sanntids GUI

🚀
