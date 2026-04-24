#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_node')

        # PID-parametere fra integrasjonsdokumentet
        self.declare_parameter('Kp_surge', 0.0)
        self.declare_parameter('Ki_surge', 0.0)

        self.declare_parameter('Kp_sway', 0.0)
        self.declare_parameter('Ki_sway', 0.0)

        self.declare_parameter('Kp_yaw', 0.0)
        self.declare_parameter('Kd_yaw', 0.0)

        self.thrust_pub = self.create_publisher(
            Wrench,
            '/thrust_demand',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('velocity_node started')
        self.get_logger().info('Publishing test /thrust_demand')

    def timer_callback(self):
        Kp_surge = self.get_parameter('Kp_surge').value
        Kp_sway = self.get_parameter('Kp_sway').value
        Kp_yaw = self.get_parameter('Kp_yaw').value

        msg = Wrench()

        # Midlertidig test:
        # Vi bruker PID-parametere direkte som output bare for å se at tuning virker.
        msg.force.x = float(Kp_surge)
        msg.force.y = float(Kp_sway)
        msg.force.z = 0.0

        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = float(Kp_yaw)

        self.thrust_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
