#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


class FakeLeak(Node):
    """
    Publiserer /leak/levels med 2 verdier (2 seksjoner).

    Kun to nivåer:
    0.0 = hvit (ingen lekkasje)
    1.0 = rød (lekkasje)
    """

    def __init__(self):
        super().__init__('fake_leak')

        self.pub = self.create_publisher(Float32MultiArray, '/leak/levels', 10)

        self.declare_parameter('rate_hz', 5.0)

        self.levels = [0.0, 0.0]

        rate_hz = float(self.get_parameter('rate_hz').value)
        dt = 1.0 / max(rate_hz, 1e-6)

        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info("FakeLeak started (2 sections, white/red only)")

    def on_timer(self):

        # tilfeldig bytte av lekkasje
        for i in range(2):

            # liten sjanse for å toggle
            if random.random() < 0.05:

                if self.levels[i] > 0.5:
                    self.levels[i] = 0.0
                else:
                    self.levels[i] = 1.0

        msg = Float32MultiArray()
        msg.data = [clamp01(v) for v in self.levels]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLeak()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
