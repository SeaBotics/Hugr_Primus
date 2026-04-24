#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ModeBridge(Node):
    def __init__(self):
        super().__init__('mode_bridge')
        self.pub = self.create_publisher(Int8, '/system_mode_status', 10)
        self.sub = self.create_subscription(Int8, '/system_command', self.cb, 10)
        self.get_logger().info('Mode bridge running')

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ModeBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
