#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def angle_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GoalPoseCmdVel(Node):
    def __init__(self):
        super().__init__('goal_pose_cmdvel')

        self.declare_parameter('fixed_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 1.8)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('goal_tolerance', 0.5)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        goal_topic = self.get_parameter('goal_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Listening for goals on {goal_topic}')
        self.get_logger().info(f'Publishing velocity on {cmd_vel_topic}')
        self.get_logger().info(f'Using TF: {self.fixed_frame} -> {self.base_frame}')

    def goal_cb(self, msg):
        self.goal = msg
        self.get_logger().info(
            f'New goal: frame={msg.header.frame_id}, x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

    def control_loop(self):
        if self.goal is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        dx = gx - x
        dy = gy - y
        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        if dist < self.get_parameter('goal_tolerance').value:
            self.cmd_pub.publish(cmd)
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = angle_wrap(desired_yaw - yaw)

        kp_linear = self.get_parameter('kp_linear').value
        kp_angular = self.get_parameter('kp_angular').value
        max_linear = self.get_parameter('max_linear').value
        max_angular = self.get_parameter('max_angular').value

        cmd.linear.x = clamp(kp_linear * dist, 0.0, max_linear)
        cmd.angular.z = clamp(kp_angular * yaw_error, -max_angular, max_angular)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GoalPoseCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
