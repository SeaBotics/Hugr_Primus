#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

class ThrusterForceMux(Node):
    def __init__(self):
        super().__init__('thruster_force_mux')

        # N per 1.0 kommando (tuning):
        self.declare_parameter('thrust_gain', 50.0)
        g = float(self.get_parameter('thrust_gain').value)

        # Map: controller_topic -> (wrench_topic, axis, sign)
        # axis: 'x' eller 'y'; sign: +1/-1
        cfg = {
            '/thruster_surge_front_controller/commands': ('/thrust/thruster_surge_front', 'x', +1),
            '/thruster_surge_rear_controller/commands':  ('/thrust/thruster_surge_rear',  'x', +1),
            '/thruster_sway_port_controller/commands':   ('/thrust/thruster_sway_port',   'y', +1),
            '/thruster_sway_starboard_controller/commands': ('/thrust/thruster_sway_starboard', 'y', -1),
        }

        self.pubs = {}
        self.subs = []
        for sub_topic, (pub_topic, axis, sign) in cfg.items():
            pub = self.create_publisher(Wrench, pub_topic, 10)
            self.pubs[sub_topic] = (pub, axis, sign)
            self.subs.append(
                self.create_subscription(Float64MultiArray, sub_topic,
                                         lambda msg, st=sub_topic, gg=g: self.cb(msg, st, gg), 10))

        self.get_logger().info('thruster_force_mux ready (gain=%.2f N/unit)' % g)

    def cb(self, msg, sub_topic, gain):
        if not msg.data:
            return
        cmd = float(msg.data[0])

        pub, axis, sign = self.pubs[sub_topic]
        w = Wrench()
        force = sign * gain * cmd
        if axis == 'x':
            w.force.x = force
        elif axis == 'y':
            w.force.y = force
        pub.publish(w)

def main():
    rclpy.init()
    n = ThrusterForceMux()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
