#!/usr/bin/env python3
import math
import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Imu


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


class FakeStatusNode(Node):
    def __init__(self):
        super().__init__("fake_status_node")

        # Existing pubs
        self.pub_mode = self.create_publisher(String, "/mode", 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_batt = self.create_publisher(Float32, "/battery_percent", 10)
        self.pub_temp = self.create_publisher(Float32, "/battery_temp", 10)
        self.pub_pos = self.create_publisher(PointStamped, "/position", 10)

        # Leak levels pub used by HugrLeakPanel
        self.pub_leak = self.create_publisher(Float32MultiArray, "/leak/levels", 10)

        # NEW: status-panel extra rows
        self.pub_sig5g = self.create_publisher(Float32, "/signal_5g_dbm", 10)
        self.pub_hull_temp = self.create_publisher(Float32, "/hull_temp", 10)

        random.seed(time.time_ns())

        self.mode = random.choice(["AUTO", "MANUAL", "AV"])

        self.x = random.uniform(-5.0, 5.0)
        self.y = random.uniform(-5.0, 5.0)
        self.z = random.uniform(-0.2, 0.2)

        self.yaw = math.radians(random.uniform(0.0, 360.0))
        self.pitch = math.radians(random.uniform(-10.0, 10.0))
        self.roll = math.radians(random.uniform(-10.0, 10.0))

        self.speed = random.uniform(0.0, 2.5)

        self.batt = random.uniform(20.0, 100.0)
        self.temp = random.uniform(20.0, 45.0)

        self.vx = random.uniform(-0.20, 0.20)
        self.vy = random.uniform(-0.20, 0.20)
        self.vz = random.uniform(-0.02, 0.02)

        self.yaw_rate = math.radians(random.uniform(-12.0, 12.0))
        self.pitch_rate = math.radians(random.uniform(-3.0, 3.0))
        self.roll_rate = math.radians(random.uniform(-3.0, 3.0))

        self.batt_drain = random.uniform(0.002, 0.02)
        self.temp_drift = random.uniform(-0.03, 0.05)

        # --- Fake acceleration parameters ---
        self.ax = random.uniform(-0.2, 0.2)
        self.ay = random.uniform(-0.2, 0.2)
        self.az = random.uniform(-0.2, 0.2)

        self.accel_noise = 0.05
        self.accel_w = 0.8
        self.accel_amp_xy = 0.6
        self.accel_amp_z = 0.2

        # ============================================================
        # Leak simulation (random event-based, no obvious pattern)
        # GUI thresholds: <0.3 gray, 0.3-0.7 orange, >0.7 red
        # ============================================================
        self.leak_levels = [0.05, 0.10, 0.08, 0.06]
        self.leak_drift = 0.015
        self.leak_noise = 0.020
        self.leak_baseline_target = [0.06, 0.10, 0.08, 0.07]

        self.leak_event_active = False
        self.leak_event_time_left = 0.0
        self.leak_event_sections = [0]
        self.leak_event_level = 0.9

        self.leak_event_rate = 0.35
        self.leak_event_min = 0.4
        self.leak_event_max = 2.2

        self.leak_orange_level = 0.60
        self.leak_red_level = 0.92

        # ============================================================
        # NEW: 5G signal (dBm) + Hull temp (°C) simulation
        # ============================================================
        # 5G dBm is usually negative: e.g. -110 (bad) .. -60 (good)
        self.sig5g_dbm = random.uniform(-105.0, -70.0)
        self.sig5g_target = random.uniform(-100.0, -65.0)
        self.sig5g_drift = 0.06          # pull toward target
        self.sig5g_noise = 1.5           # random jitter per tick (dBm)
        self.sig5g_target_change_p = 0.01  # chance to pick a new target

        # Hull temp (°C)
        self.hull_temp = random.uniform(5.0, 25.0)
        self.hull_target = random.uniform(5.0, 35.0)
        self.hull_drift = 0.03
        self.hull_noise = 0.25
        self.hull_target_change_p = 0.008

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info("Fake status running (IMU accel + random leak + 5G dBm + hull temp). Ctrl+C to stop.")

    def _maybe_start_leak_event(self):
        p = self.leak_event_rate * self.dt
        if random.random() < p:
            self.leak_event_active = True
            self.leak_event_time_left = random.uniform(self.leak_event_min, self.leak_event_max)

            k = random.choice([1, 1, 2, 2, 3])
            self.leak_event_sections = random.sample([0, 1, 2, 3], k=k)

            self.leak_event_level = self.leak_red_level if (random.random() < 0.45) else self.leak_orange_level

    def _update_leaks(self):
        for i in range(4):
            target = self.leak_baseline_target[i]
            self.leak_levels[i] += (target - self.leak_levels[i]) * self.leak_drift
            self.leak_levels[i] += random.uniform(-self.leak_noise, self.leak_noise)
            self.leak_levels[i] = clamp01(self.leak_levels[i])

        if not self.leak_event_active:
            self._maybe_start_leak_event()
        else:
            self.leak_event_time_left -= self.dt

            for j in self.leak_event_sections:
                self.leak_levels[j] += (self.leak_event_level - self.leak_levels[j]) * 0.35
                self.leak_levels[j] += random.uniform(-0.04, 0.04)
                self.leak_levels[j] = clamp01(self.leak_levels[j])

            if random.random() < 0.08:
                candidate = random.choice([0, 1, 2, 3])
                if candidate in self.leak_event_sections and len(self.leak_event_sections) > 1:
                    self.leak_event_sections.remove(candidate)
                elif candidate not in self.leak_event_sections and len(self.leak_event_sections) < 3:
                    self.leak_event_sections.append(candidate)

            if self.leak_event_time_left <= 0.0:
                self.leak_event_active = False
                self.leak_event_time_left = 0.0
                self.leak_event_sections = [0]

                for i in range(4):
                    self.leak_baseline_target[i] = clamp01(self.leak_baseline_target[i] + random.uniform(-0.03, 0.03))
                    self.leak_baseline_target[i] = min(self.leak_baseline_target[i], 0.18)

    def _update_5g_and_hull(self):
        # Occasionally choose new targets (simulates moving coverage / environment)
        if random.random() < self.sig5g_target_change_p:
            self.sig5g_target = random.uniform(-105.0, -65.0)

        if random.random() < self.hull_target_change_p:
            self.hull_target = random.uniform(2.0, 40.0)

        # Pull toward target + noise
        self.sig5g_dbm += (self.sig5g_target - self.sig5g_dbm) * self.sig5g_drift
        self.sig5g_dbm += random.uniform(-self.sig5g_noise, self.sig5g_noise)
        self.sig5g_dbm = clamp(self.sig5g_dbm, -120.0, -40.0)

        self.hull_temp += (self.hull_target - self.hull_temp) * self.hull_drift
        self.hull_temp += random.uniform(-self.hull_noise, self.hull_noise)
        self.hull_temp = clamp(self.hull_temp, -10.0, 80.0)

    def tick(self):
        # --- Position ---
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        if abs(self.x) > 8.0:
            self.vx *= -1.0
        if abs(self.y) > 8.0:
            self.vy *= -1.0
        self.z = clamp(self.z, -0.3, 0.3)

        # --- Orientation ---
        self.yaw += self.yaw_rate * self.dt
        self.pitch += self.pitch_rate * self.dt
        self.roll += self.roll_rate * self.dt
        self.yaw = (self.yaw + 2.0 * math.pi) % (2.0 * math.pi)

        # --- Battery ---
        self.batt = clamp(self.batt - self.batt_drain, 0.0, 100.0)
        self.temp = clamp(self.temp + self.temp_drift, -20.0, 80.0)

        # --- Speed ---
        self.speed = clamp(self.speed + random.uniform(-0.03, 0.03), 0.0, 3.5)

        # --- Occasional mode change ---
        if random.random() < 0.002:
            self.mode = random.choice(["AUTO", "MANUAL", "AV"])

        # --- Fake linear acceleration ---
        tnow = self.get_clock().now().nanoseconds * 1e-9
        self.ax = self.accel_amp_xy * math.sin(self.accel_w * tnow) + random.uniform(-self.accel_noise, self.accel_noise)
        self.ay = self.accel_amp_xy * math.cos(self.accel_w * tnow) + random.uniform(-self.accel_noise, self.accel_noise)
        self.az = self.accel_amp_z  * math.sin(0.5 * self.accel_w * tnow) + random.uniform(-self.accel_noise, self.accel_noise)

        # --- Leak update (random) ---
        self._update_leaks()

        # --- NEW: 5G + hull temp update ---
        self._update_5g_and_hull()

        # --- Publish mode ---
        msg_mode = String()
        msg_mode.data = self.mode
        self.pub_mode.publish(msg_mode)

        # --- Publish cmd_vel ---
        msg_tw = Twist()
        msg_tw.linear.x = float(self.speed)
        msg_tw.linear.y = 0.0
        msg_tw.linear.z = 0.0
        self.pub_twist.publish(msg_tw)

        # --- Publish IMU ---
        msg_imu = Imu()
        qx, qy, qz, qw = rpy_to_quat(self.roll, self.pitch, self.yaw)
        msg_imu.orientation.x = float(qx)
        msg_imu.orientation.y = float(qy)
        msg_imu.orientation.z = float(qz)
        msg_imu.orientation.w = float(qw)

        msg_imu.angular_velocity.x = float(self.roll_rate)
        msg_imu.angular_velocity.y = float(self.pitch_rate)
        msg_imu.angular_velocity.z = float(self.yaw_rate)

        msg_imu.linear_acceleration.x = float(self.ax)
        msg_imu.linear_acceleration.y = float(self.ay)
        msg_imu.linear_acceleration.z = float(self.az)

        self.pub_imu.publish(msg_imu)

        # --- Publish battery ---
        msg_b = Float32()
        msg_b.data = float(self.batt)
        self.pub_batt.publish(msg_b)

        msg_t = Float32()
        msg_t.data = float(self.temp)
        self.pub_temp.publish(msg_t)

        # --- Publish position ---
        msg_p = PointStamped()
        msg_p.header.frame_id = "map"
        msg_p.header.stamp = self.get_clock().now().to_msg()
        msg_p.point.x = float(self.x)
        msg_p.point.y = float(self.y)
        msg_p.point.z = float(self.z)
        self.pub_pos.publish(msg_p)

        # --- Publish leak levels (Float32MultiArray, 4 values) ---
        msg_leak = Float32MultiArray()
        msg_leak.data = [float(clamp01(v)) for v in self.leak_levels[:4]]
        self.pub_leak.publish(msg_leak)

        # --- NEW: Publish 5G signal + hull temp ---
        msg_sig = Float32()
        msg_sig.data = float(self.sig5g_dbm)
        self.pub_sig5g.publish(msg_sig)

        msg_hull = Float32()
        msg_hull.data = float(self.hull_temp)
        self.pub_hull_temp.publish(msg_hull)


def main():
    rclpy.init()
    node = FakeStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
