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
- Kamerafeed
- Operasjonsmodus

---

# Workspace Struktur

Prosjektet er organisert som et ROS 2 workspace med følgende struktur:

```text
ros2_ws/
└── src/
    └── Hugr_Primus_gui/
        └── seabotics_ws/
            └── src/
                ├── gui/
                │   ├── hugr_rviz_panel/
                │   └── hugr_fake_status/
                └── trimaran_description/
```

### Forklaring

- hugr_rviz_panel  
  RViz plugins for GUI (status, kompass, gyro, leak, mode)

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
| HugrCameraSubscriber | Kamerafeed-visualisering | 
| HugrModePanel | Viser aktiv operasjonsmodus |

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

Mode vises også i et eget Mode Panel for tydeligere visualisering og debugging.

### Batterifarge

| Batteri % | Farge |
|----------|------|
| 100 – 51 % | Grønn |
| 50 – 21 % | Gul |
| 20 – 0 % | Rød |

### Mode fargekode

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

### Topic

/leak/levels

Type:

std_msgs/Float32MultiArray

Data representerer lekkasjestatus per seksjon.

---

# HugrModePanel

Mode-panelet viser nåværende operasjonsmodus til fartøyet i sanntid.

Panelet brukes for å visualisere beslutninger fra høyere nivå i systemet, som for eksempel Behavior Tree (BT) eller kontrollsystemet.

### Topic

/mode

Type:

std_msgs/String

### Funksjonalitet

- Viser aktiv modus i GUI
- Fargekoding for rask tolkning:

| Mode | Farge |
|-----|------|
| Kill Switch / Emergency | Rød |
| Manual | Gul |
| Autonomous | Grønn |

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
| /Image_Compressed | Camera/Image_raw | Kamerafeed |

---

# Build

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

---

# Launch

ros2 launch trimaran_description display.launch.py

Start RViz:

rviz2

Legg til paneler:

Panels → Add New Panel

Velg:

hugr_rviz_panel/HugrStatusPanel  
hugr_rviz_panel/HugrCompassPanel  
hugr_rviz_panel/HugrLeakPanel
hugr_rviz_panel/HugrCameraPanel  

Legg til display:

Displays → Add  

hugr_rviz_panel/HugrGyroDisplay


---

# 🖥️ Bruk av GUI i RViz

Etter launch kan GUI-panelene legges til manuelt i RViz.

### Start RViz

rviz2

---

## ➕ Legg til paneler

I RViz:

Panels → Add New Panel

Legg til:

| Panel | Funksjon |
|------|--------|
| Hugr Status Panel | Systemstatus |
| Hugr Mode Panel | Operasjonsmodus |
| Hugr Leak Panel | Lekkasjedeteksjon |
| Hugr Compass Panel | Heading |

---

## 🧭 Legg til Gyro Display

Gyro er en Display, ikke et panel.

I RViz:

Add → By display type → HugrGyroDisplay

Viser:
- 3D orientering
- Pitch / Roll / Yaw

---

## ⚙️ Viktige innstillinger

Sett riktig Fixed Frame:

world

---

## 📡 Krav til topics

For at GUI skal fungere må følgende topics publiseres:

- /imu/data
- /cmd_vel
- /battery_percent
- /battery_temp
- /position
- /mode
- /leak/levels

---

## 🧪 Testing uten hardware

ros2 run hugr_fake_status fake_status

---

# 🖥️ Bruk av GUI i RViz

Etter launch kan GUI-panelene legges til manuelt i RViz.

### Start RViz

rviz2

---

## ➕ Legg til paneler

I RViz:

Panels → Add New Panel

Legg til:

| Panel | Funksjon |
|------|--------|
| Hugr Status Panel | Systemstatus |
| Hugr Mode Panel | Operasjonsmodus |
| Hugr Leak Panel | Lekkasjedeteksjon |
| Hugr Compass Panel | Heading |

---

## 🧭 Legg til Gyro Display

Gyro er en Display, ikke et panel.

I RViz:

Add → By display type → HugrGyroDisplay

Viser:
- 3D orientering
- Pitch / Roll / Yaw

---

## ⚙️ Viktige innstillinger

Sett riktig Fixed Frame:

world

---

## 📡 Krav til topics

For at GUI skal fungere må følgende topics publiseres:

- /imu/data
- /cmd_vel
- /battery_percent
- /battery_temp
- /position
- /mode
- /leak/levels

---

## 🧪 Testing uten hardware

ros2 run hugr_fake_status fake_status

---

# 🖥️ Bruk av GUI i RViz

Etter launch kan GUI-panelene legges til manuelt i RViz.

### Start RViz

rviz2

---

## ➕ Legg til paneler

I RViz:

Panels → Add New Panel

Legg til:

| Panel | Funksjon |
|------|--------|
| Hugr Status Panel | Systemstatus |
| Hugr Mode Panel | Operasjonsmodus |
| Hugr Leak Panel | Lekkasjedeteksjon |
| Hugr Compass Panel | Heading |

---

## 🧭 Legg til Gyro Display

Gyro er en Display, ikke et panel.

I RViz:

Add → By display type → HugrGyroDisplay

Viser:
- 3D orientering
- Pitch / Roll / Yaw

---

## ⚙️ Viktige innstillinger

Sett riktig Fixed Frame:

world

---

## 📡 Krav til topics

For at GUI skal fungere må følgende topics publiseres:

- /imu/data
- /cmd_vel
- /battery_percent
- /battery_temp
- /position
- /mode
- /leak/levels

---

## 🧪 Testing uten hardware

ros2 run hugr_fake_status fake_status

---

# Fake Status vs Real Data

Statuspanelet er satt opp for reelle sensordata.

Ved testing uten hardware brukes fake node som publiserer samme topics.

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

### Støttet hardware

- Jetson Orin Nano
- Ubuntu PC
- WSL2

---

# SeaBotics

Prosjekt utviklet av SeaBotics studentforening.

🚀


