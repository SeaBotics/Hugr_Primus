# 🧭 Hugr Primus – SeaBotics ROS 2 GUI

GUI og visualiseringssystem for Hugr-Primus utviklet av SeaBotics.

Systemet er bygget på ROS 2 Humble + RViz plugins og brukes til å overvåke status fra sensorer på fartøyet.

GUI-en viser blant annet:

- Batteristatus
- Navigasjon og IMU-data
- Fart og posisjon
- Leak detection i skroget
- Kompass
- Gyro visualisering

---

# Workspace Struktur

Prosjektet er organisert som et ROS 2 workspace med følgende struktur:

ros2_ws/
└── src/
    └── Hugr_Primus_gui/
        └── seabotics_ws/
            └── src/
                ├── gui/
                │   ├── hugr_rviz_panel/
                │   └── hugr_fake_status/
                └── trimaran_description/

### Forklaring

- hugr_rviz_panel  
  RViz plugins for GUI (status, kompass, gyro, leak)

- hugr_fake_status  
  Fake sensornoder for testing uten hardware

- trimaran_description  
  URDF-modell av båten og launch-filer

---

# Systemoversikt

GUI består av flere RViz plugins.

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

### Batterifarge

| Batteri % | Farge |
|----------|------|
| 100 – 51 % | Grønn |
| 50 – 21 % | Gul |
| 20 – 0 % | Rød |

Mode vises med fargekode:

| Mode | Farge |
|-----|------|
| Kill Switch | Rød |
| Manual | Gul |
| Autonomous | Grønn |

---

# HugrLeakPanel

Leak-panelet viser en visualisering av skroget.

Skroget er delt i 2 seksjoner.

| Verdi | Betydning |
|------|------|
| 0 | Ingen lekkasje |
| 1 | Lekkasjedeteksjon |

Panelet abonnerer på topic:

/leak/levels

Type:

std_msgs/Float32MultiArray

Data representerer lekkasjestatus per seksjon.

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

# Build

cd ~/ros2_ws  
source /opt/ros/humble/setup.bash  

colcon build --symlink-install  

source install/setup.bash  

---

# Launch

ros2 launch trimaran_description display.launch.py

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

Prosjekt utviklet av SeaBotics studentforening.

🚀
