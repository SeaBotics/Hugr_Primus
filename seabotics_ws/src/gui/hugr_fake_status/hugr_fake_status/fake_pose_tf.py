#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakePoseTF(Node):
    def __init__(self):
        super().__init__('fake_pose_tf')

        self.br = TransformBroadcaster(self)

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.timer = self.create_timer(0.05, self.publish_tf)

        self.get_logger().info(
            'fake_pose_tf started: publishing dynamic world -> base_link from /imu/data'
        )

    def imu_cb(self, msg: Imu):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = float(self.qx)
        t.transform.rotation.y = float(self.qy)
        t.transform.rotation.z = float(self.qz)
        t.transform.rotation.w = float(self.qw)

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FakePoseTF()
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


if __name__ == '__main__':
    main()
