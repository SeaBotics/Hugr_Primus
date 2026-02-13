# 🚢 Hugr Primus
### SeaBotics Autonomous Surface Vessel GUI System

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Jetson%20Orin%20Nano-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![License](https://img.shields.io/badge/License-Internal-lightgrey)

---

## 📖 Overview

Hugr Primus is the official SeaBotics GUI framework for visualization and system monitoring of our autonomous surface vessel.

The system consists of:

- 🖥 **RViz 2 Status Panel (Qt)**
- 🧭 **Compass Panel**
- 🔴🟢🔵 **3D Gyro Display (Yaw / Pitch / Roll)**
- 🤖 **Fake Status Node (Python test publisher)**

Designed for:

- Development in WSL / Ubuntu
- Deployment on Jetson Orin Nano
- Future integration with real IMU, EKF and control stack

---

## 🏗 System Architecture

```
hugr_fake_status (Python Node)
        │
        ▼
       ROS 2 Topics
        │
        ▼
RViz 2 Plugins (C++)
 ├── Status Panel
 ├── Compass Panel
 └── Gyro Display (OGRE 3D)
```

---

## 📂 Workspace Structure

```
~/ros2_ws
 └── src
      └── Hugr_Primus
           ├── hugr_rviz_panel
           └── hugr_fake_status
```

Repository:
```
SeaBotics/Hugr_Primus
Branch: gui_Panel
```

---

## 🛠 Requirements

### OS
Ubuntu 22.04

### Hardware
Jetson Orin Nano (recommended)

### ROS
ROS 2 Humble

---

## 📦 Installation

### 1️⃣ System Dependencies

```bash
sudo apt update
sudo apt install -y   git openssh-client   build-essential cmake pkg-config   python3-pip python3-venv   python3-colcon-common-extensions   python3-rosdep   qtbase5-dev qttools5-dev qttools5-dev-tools   libqt5svg5-dev   ros-humble-desktop   ros-humble-rviz2   ros-humble-pluginlib   ros-humble-tf2   ros-humble-tf2-ros
```

---

### 2️⃣ Setup ROS Environment

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 3️⃣ Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone git@github.com:SeaBotics/Hugr_Primus.git
cd Hugr_Primus
git checkout gui_Panel
```

---

### 4️⃣ Build Workspace

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Running the System

### Start Fake Data

```bash
ros2 run hugr_fake_status fake_status_node
```

### Start RViz

```bash
rviz2
```

Add the following plugins:

- Hugr Status Panel
- Hugr Compass Panel
- Hugr Gyro Display

---

## 📡 ROS Topics

| Topic | Type |
|-------|------|
| `/imu/data` | sensor_msgs/msg/Imu |
| `/cmd_vel` | geometry_msgs/msg/Twist |
| `/mode` | std_msgs/msg/String |
| `/battery` | std_msgs/msg/Float32 |
| `/battery_temp` | std_msgs/msg/Float32 |
| `/position` | geometry_msgs/msg/Point |

---

## ⚡ Jetson Performance Mode

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

---

## 🧪 Development Workflow

Create new feature branch:

```bash
git checkout -b feature/<feature_name>
```

Push branch:

```bash
git push -u origin feature/<feature_name>
```

Open Pull Request → `gui_Panel`

---

## 🔐 SeaBotics Development Rules

- No global state
- All ROS subscriptions inside constructors
- UI updates via QTimer
- Use `std::mutex` for shared data
- GUI is never safety‑critical

---

## 🚀 Roadmap

- Launch files
- Parameter YAML configuration
- EKF integration
- State machine node
- Thruster allocation
- Watchdog / failsafe system

---

SeaBotics – University of Agder
