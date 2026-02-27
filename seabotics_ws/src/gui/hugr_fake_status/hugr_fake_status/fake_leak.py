#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

class FakeLeak(Node):
    """
    Publiserer /leak/levels (Float32MultiArray) med 4 verdier i [0,1].
    Pattern: rolig sinus + litt offset per seksjon, slik at du ser endringer i GUI.
    """
    def __init__(self):
        super().__init__('fake_leak')

        self.pub = self.create_publisher(Float32MultiArray, '/leak/levels', 10)

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('mode', 'wave')  # wave | fixed | ramp
        self.declare_parameter('fixed', [0.0, 0.2, 0.6, 0.9])  # brukt i fixed-mode
        self.declare_parameter('ramp_speed', 0.08)  # brukt i ramp-mode (per sekund)

        self.t = 0.0
        self.levels = [0.0, 0.0, 0.0, 0.0]

        rate_hz = float(self.get_parameter('rate_hz').value)
        dt = 1.0 / max(rate_hz, 1e-6)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(f"FakeLeak started: publishing /leak/levels at {rate_hz:.1f} Hz")

    def on_timer(self):
        rate_hz = float(self.get_parameter('rate_hz').value)
        dt = 1.0 / max(rate_hz, 1e-6)
        self.t += dt

        mode = str(self.get_parameter('mode').value).lower()

        msg = Float32MultiArray()

        if mode == 'fixed':
            vals = list(self.get_parameter('fixed').value)
            # sørg for 4 elementer
            vals = (vals + [0.0, 0.0, 0.0, 0.0])[:4]
            msg.data = [clamp01(float(v)) for v in vals]

        elif mode == 'ramp':
            speed = float(self.get_parameter('ramp_speed').value)
            # ramp hver seksjon litt forskjellig
            for i in range(4):
                self.levels[i] = clamp01(self.levels[i] + speed * dt * (1.0 if i % 2 == 0 else 0.6))
            msg.data = self.levels

        else:
            # wave (default): litt forskjellig fase per seksjon
            base = 0.5
            amp = 0.5
            vals = [
                base + amp * 0.5 * (math.sin(2.0*self.t + 0.0) + 1.0) - 0.25,
                base + amp * 0.5 * (math.sin(2.0*self.t + 1.2) + 1.0) - 0.30,
                base + amp * 0.5 * (math.sin(2.0*self.t + 2.1) + 1.0) - 0.10,
                base + amp * 0.5 * (math.sin(2.0*self.t + 3.0) + 1.0) - 0.05,
            ]
            msg.data = [clamp01(v) for v in vals]

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
