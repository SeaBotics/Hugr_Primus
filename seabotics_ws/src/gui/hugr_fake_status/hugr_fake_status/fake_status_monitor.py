#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String

class FakeStatusMonitor(Node):

    def __init__(self):
        super().__init__('fake_status_monitor')

        # Leak
        self.create_subscription(
            Float32MultiArray,
            '/leak/levels',
            self.leak_callback,
            10)

        # Eksempler – legg til de topicene du faktisk bruker:
        self.create_subscription(
            Float32,
            '/heading',
            self.heading_callback,
            10)

        self.create_subscription(
            Float32,
            '/battery',
            self.battery_callback,
            10)

        self.create_subscription(
            String,
            '/status',
            self.status_callback,
            10)

        self.get_logger().info("FakeStatusMonitor started")

    # ---- Callbacks ----

    def leak_callback(self, msg):
        levels = [round(v, 2) for v in msg.data]
        print(f"LEAK LEVELS: {levels}")

    def heading_callback(self, msg):
        print(f"Heading: {msg.data:.2f}")

    def battery_callback(self, msg):
        print(f"Battery: {msg.data:.1f}%")

    def status_callback(self, msg):
        print(f"Status: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeStatusMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
