#!/usr/bin/env python3
import math
import random
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
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


class FakeStatusNode(Node):
    def __init__(self):
        super().__init__("fake_status_node")

        self.pub_mode = self.create_publisher(String, "/mode", 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_batt = self.create_publisher(Float32, "/battery_percent", 10)
        self.pub_temp = self.create_publisher(Float32, "/battery_temp", 10)
        self.pub_pos = self.create_publisher(PointStamped, "/position", 10)

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

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info("Fake status running: random init each start. Ctrl+C to stop.")

    def tick(self):
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        if abs(self.x) > 8.0:
            self.vx *= -1.0
        if abs(self.y) > 8.0:
            self.vy *= -1.0
        self.z = clamp(self.z, -0.3, 0.3)

        self.yaw += self.yaw_rate * self.dt
        self.pitch += self.pitch_rate * self.dt
        self.roll += self.roll_rate * self.dt

        self.yaw = (self.yaw + 2.0 * math.pi) % (2.0 * math.pi)

        self.batt = clamp(self.batt - self.batt_drain, 0.0, 100.0)
        self.temp = clamp(self.temp + self.temp_drift, -20.0, 80.0)

        self.speed = clamp(self.speed + random.uniform(-0.03, 0.03), 0.0, 3.5)

        if random.random() < 0.002:
            self.mode = random.choice(["AUTO", "MANUAL", "AV"])

        msg_mode = String()
        msg_mode.data = self.mode
        self.pub_mode.publish(msg_mode)

        msg_tw = Twist()
        msg_tw.linear.x = float(self.speed)
        msg_tw.linear.y = 0.0
        msg_tw.linear.z = 0.0
        self.pub_twist.publish(msg_tw)

        msg_imu = Imu()
        qx, qy, qz, qw = rpy_to_quat(self.roll, self.pitch, self.yaw)
        msg_imu.orientation.x = float(qx)
        msg_imu.orientation.y = float(qy)
        msg_imu.orientation.z = float(qz)
        msg_imu.orientation.w = float(qw)
        self.pub_imu.publish(msg_imu)

        msg_b = Float32()
        msg_b.data = float(self.batt)
        self.pub_batt.publish(msg_b)

        msg_t = Float32()
        msg_t.data = float(self.temp)
        self.pub_temp.publish(msg_t)

        msg_p = PointStamped()
        msg_p.header.frame_id = "map"
        msg_p.header.stamp = self.get_clock().now().to_msg()
        msg_p.point.x = float(self.x)
        msg_p.point.y = float(self.y)
        msg_p.point.z = float(self.z)
        self.pub_pos.publish(msg_p)


def main():
    # Robust shutdown on Ctrl+C (avoid "rcl_shutdown already called")
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
            # If shutdown already happened, ignore
            pass


if __name__ == "__main__":
    main()
