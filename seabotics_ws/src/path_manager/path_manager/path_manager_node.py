#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseArray


class PathManagerNode(Node):
    def __init__(self):
        super().__init__('path_manager_node')

        # Parameters
        self.declare_parameter('gps_target_topic', '/path_manager/gps_target')
        self.declare_parameter('gate_midpoints_topic', '/perception/gate_midpoints')
        self.declare_parameter('position_topic', '/state/position')
        self.declare_parameter('active_target_topic', '/guidance/active_midpoint')

        self.declare_parameter('publish_period', 0.2)
        self.declare_parameter('min_forward_x', 0.5)
        self.declare_parameter('vision_hold_time', 1.0)
        self.declare_parameter('gps_reached_radius', 2.0)

        gps_target_topic = self.get_parameter('gps_target_topic').value
        gate_midpoints_topic = self.get_parameter('gate_midpoints_topic').value
        position_topic = self.get_parameter('position_topic').value
        active_target_topic = self.get_parameter('active_target_topic').value

        publish_period = float(self.get_parameter('publish_period').value)
        self.min_forward_x = float(self.get_parameter('min_forward_x').value)
        self.vision_hold_time = float(self.get_parameter('vision_hold_time').value)
        self.gps_reached_radius = float(self.get_parameter('gps_reached_radius').value)

        # State
        self.gps_target: Optional[Tuple[float, float]] = None
        self.current_position: Optional[Tuple[float, float]] = None

        # Gate midpoints are assumed to be in BODY FRAME
        self.detected_gate_targets_body: List[Tuple[float, float]] = []

        self.current_mode = 'gps'   # 'gps' or 'vision'
        self.current_target: Optional[Tuple[float, float]] = None

        self.last_vision_time: Optional[float] = None

        # Subscribers
        self.gps_sub = self.create_subscription(
            PointStamped,
            gps_target_topic,
            self.gps_target_callback,
            10
        )

        self.gates_sub = self.create_subscription(
            PoseArray,
            gate_midpoints_topic,
            self.gate_midpoints_callback,
            10
        )

        self.position_sub = self.create_subscription(
            PointStamped,
            position_topic,
            self.position_callback,
            10
        )

        # Publisher
        self.active_target_pub = self.create_publisher(
            PointStamped,
            active_target_topic,
            10
        )

        # Timer
        self.timer = self.create_timer(publish_period, self.timer_callback)

        self.get_logger().info('Path manager node started.')
        self.get_logger().info(f'GPS target topic: {gps_target_topic}')
        self.get_logger().info(f'Gate midpoints topic: {gate_midpoints_topic}')
        self.get_logger().info(f'Position topic: {position_topic}')
        self.get_logger().info(f'Active target topic: {active_target_topic}')

    def gps_target_callback(self, msg: PointStamped):
        self.gps_target = (msg.point.x, msg.point.y)

    def gate_midpoints_callback(self, msg: PoseArray):
        self.detected_gate_targets_body = [
            (pose.position.x, pose.position.y)
            for pose in msg.poses
        ]

        if len(self.detected_gate_targets_body) > 0:
            self.last_vision_time = self.get_clock().now().nanoseconds * 1e-9

    def position_callback(self, msg: PointStamped):
        self.current_position = (msg.point.x, msg.point.y)

    def filter_valid_gates(self, gates_body: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        # Keep only gates in front of the boat
        return [(x, y) for (x, y) in gates_body if x > self.min_forward_x]

    def choose_best_gate(self, gates_body: List[Tuple[float, float]]) -> Tuple[float, float]:
        # Simplest rule:
        # among gates in front of the boat, choose the nearest one
        return min(gates_body, key=lambda p: math.hypot(p[0], p[1]))

    def gps_target_reached(self) -> bool:
        if self.gps_target is None or self.current_position is None:
            return False

        gx, gy = self.gps_target
        px, py = self.current_position
        return math.hypot(gx - px, gy - py) < self.gps_reached_radius

    def vision_recently_available(self) -> bool:
        if self.last_vision_time is None:
            return False

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return (now_sec - self.last_vision_time) < self.vision_hold_time

    def publish_active_target(self, x: float, y: float, frame_id: str):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.active_target_pub.publish(msg)

    def timer_callback(self):
        valid_gates = self.filter_valid_gates(self.detected_gate_targets_body)

        # Priority 1: valid vision gates
        if len(valid_gates) > 0:
            best_gate = self.choose_best_gate(valid_gates)
            self.current_mode = 'vision'
            self.current_target = best_gate

            # Publish in body frame because LOS v1 expects relative target
            self.publish_active_target(best_gate[0], best_gate[1], 'base_link')

            self.get_logger().debug(
                f'Mode=vision target=({best_gate[0]:.2f}, {best_gate[1]:.2f})'
            )
            return

        # Optional small hysteresis: remain in vision briefly if detections just disappeared
        if self.current_mode == 'vision' and self.vision_recently_available() and self.current_target is not None:
            tx, ty = self.current_target
            self.publish_active_target(tx, ty, 'base_link')
            return

        # Priority 2: GPS fallback
        if self.gps_target is not None:
            self.current_mode = 'gps'
            self.current_target = self.gps_target

            # Publish in map frame because GPS target is global
            self.publish_active_target(self.gps_target[0], self.gps_target[1], 'map')

            self.get_logger().debug(
                f'Mode=gps target=({self.gps_target[0]:.2f}, {self.gps_target[1]:.2f})'
            )
            return

        # No target available
        self.current_target = None


def main(args=None):
    rclpy.init(args=args)
    node = PathManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()