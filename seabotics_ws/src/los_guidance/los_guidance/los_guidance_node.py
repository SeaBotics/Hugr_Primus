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
        self.declare_parameter('min_forward_x', 0.05)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_forward_x = self.get_parameter('min_forward_x').get_parameter_value().double_value
        
        # Publisher
        self.heading_pub = self.create_publisher(Float64, output_topic, 10)

        # subscriber
        self.target_sub = self.create_subscription(
            PointStamped,
            input_topic,
            self.target_callback,
            10
        )

        self.get_logger().info(f'LOS guidance node started.')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing desired heading to: {output_topic}')
    
    def target_callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y

        # Basic guard against unstable commands when target is too close
        if abs(x) < self.min_forward_x and abs(y) < self.min_forward_x:
            self.get_logger().warn('Target point too close to origin; skipping heading update')
            return
        
        # Minimal LOS
        desired_heading = math.atan2(y, x)

        heading_msg = Float64
        heading_msg.data = desired_heading
        self.heading_pub.publish(heading_msg)

        self.get_logger().info(
            f'Target: x={x:.3f}, y={y:.3f} -> desired_heading={desired_heading:.3f} rad'
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