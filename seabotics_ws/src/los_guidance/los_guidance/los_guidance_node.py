#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class LOSGuidanceNode(Node):
    def __init__(self):
        super().__init__('los_guidance_node')

        # Parametre
        self.declare_parameter('input_topic', '/guidance/active_midpoint')
        self.declare_parameter('output_topic', '/guidance/desired_heading')
        self.declare_parameter('position_topic', '/state/position')
        self.declare_parameter('yaw_topic', '/state/yaw')
        self.declare_parameter('min_forward_x', 0.05)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        position_topic = self.get_parameter('position_topic').value
        yaw_topic = self.get_parameter('yaw_topic').value
        self.min_forward_x = float(self.get_parameter('min_forward_x').value)

        # Intern state
        self.current_position = None   # (x, y) in map
        self.current_yaw = None        # yaw in rad, map/world frame

        # Publisher
        self.heading_pub = self.create_publisher(Float64, output_topic, 10)

        # Subscribers
        self.target_sub = self.create_subscription(
            PointStamped,
            input_topic,
            self.target_callback,
            10
        )

        self.position_sub = self.create_subscription(
            PointStamped,
            position_topic,
            self.position_callback,
            10
        )

        self.yaw_sub = self.create_subscription(
            Float64,
            yaw_topic,
            self.yaw_callback,
            10
        )

        self.get_logger().info('LOS guidance node started.')
        self.get_logger().info(f'Subscribing target: {input_topic}')
        self.get_logger().info(f'Subscribing position: {position_topic}')
        self.get_logger().info(f'Subscribing yaw: {yaw_topic}')
        self.get_logger().info(f'Publishing desired heading to: {output_topic}')

    def position_callback(self, msg: PointStamped):
        self.current_position = (msg.point.x, msg.point.y)

    def yaw_callback(self, msg: Float64):
        self.current_yaw = msg.data

    def wrap_to_pi(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def target_callback(self, msg: PointStamped):
        frame_id = msg.header.frame_id.strip()
        target_x = msg.point.x
        target_y = msg.point.y

        # Case 1: target already in body frame
        if frame_id == 'base_link':
            if abs(target_x) < self.min_forward_x and abs(target_y) < self.min_forward_x:
                self.get_logger().warn('Target point too close to origin; skipping heading update')
                return

            desired_heading = math.atan2(target_y, target_x)

        # Case 2: target in map frame
        elif frame_id == 'map':
            if self.current_position is None or self.current_yaw is None:
                self.get_logger().warn('Missing position/yaw for map-frame target; skipping heading update')
                return

            px, py = self.current_position

            dx = target_x - px
            dy = target_y - py

            # Absolute bearing in map frame
            target_bearing = math.atan2(dy, dx)

            # Desired heading relative to current yaw
            desired_heading = self.wrap_to_pi(target_bearing - self.current_yaw)

        else:
            self.get_logger().warn(f'Unsupported frame_id "{frame_id}". Expected "base_link" or "map".')
            return

        heading_msg = Float64()
        heading_msg.data = desired_heading
        self.heading_pub.publish(heading_msg)

        self.get_logger().info(
            f'frame={frame_id}, target=({target_x:.3f}, {target_y:.3f}) -> desired_heading={desired_heading:.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()