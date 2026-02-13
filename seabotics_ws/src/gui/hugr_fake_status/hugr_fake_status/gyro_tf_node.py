#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GyroTF(Node):
    def __init__(self):
        super().__init__("gyro_tf_node")

        self.br = TransformBroadcaster(self)

        self.parent_frame = "map"
        self.child_frame = "gyro_link"

        self.pos = (0.0, 0.0, 0.0)

        self.sub_imu = self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.sub_pos = self.create_subscription(PointStamped, "/position", self.pos_cb, 10)

        self.get_logger().info("Gyro TF broadcaster running. Publishing TF map -> gyro_link")

    def pos_cb(self, msg: PointStamped):
        self.pos = (msg.point.x, msg.point.y, msg.point.z)

    def imu_cb(self, msg: Imu):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # translation (bruk pos hvis du vil, ellers 0)
        t.transform.translation.x = float(self.pos[0])
        t.transform.translation.y = float(self.pos[1])
        t.transform.translation.z = float(self.pos[2])

        # rotation = IMU orientation
        t.transform.rotation = msg.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = GyroTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
