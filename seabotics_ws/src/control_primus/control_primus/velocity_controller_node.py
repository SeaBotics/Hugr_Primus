#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8 # NYTT: For å lytte på modus

class VelocityControllerNode(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.declare_parameter('Kp_surge', 5.0)
        self.declare_parameter('Ki_surge', 0.5)
        self.declare_parameter('Kp_sway', 5.0)
        self.declare_parameter('Ki_sway', 0.5)
        self.declare_parameter('Kp_yaw', 3.0)
        self.declare_parameter('Kd_yaw', 1.5)

        # ENDRET: Lytter nå på '/cmd_vel_auto' (som kommer fra Mode Manager, ikke direkte fra Nav2)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel_auto', self.cmd_vel_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered_local', self.odom_callback, 10)
        
        # NYTT: Lytter på System Modus
        self.sub_mode = self.create_subscription(Int8, '/system_mode_status', self.mode_callback, 10)
        
        self.pub_thrust = self.create_publisher(Wrench, '/thrust_demand', 10)

        self.current_mode = 0 # Starter i Killswitch
        self.ref_surge = 0.0
        self.ref_sway = 0.0
        self.ref_yaw_rate = 0.0
        self.act_surge = 0.0
        self.act_sway = 0.0
        self.act_yaw_rate = 0.0

        self.int_surge = 0.0
        self.int_sway = 0.0
        self.int_yaw = 0.0
        self.prev_err_yaw = 0.0

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Velocity Controller er klar. Venter på Auto-modus.")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def cmd_vel_callback(self, msg):
        self.ref_surge = msg.linear.x
        self.ref_sway = msg.linear.y
        self.ref_yaw_rate = msg.angular.z

    def odom_callback(self, msg):
        self.act_surge = msg.twist.twist.linear.x
        self.act_sway = msg.twist.twist.linear.y
        self.act_yaw_rate = msg.twist.twist.angular.z

    def control_loop(self):
        # SIKKERHET & RYDDIGHET: Hvis vi ikke er i Auto, skal PID sove og glemme alt.
        if self.current_mode != 2:
            self.int_surge = 0.0
            self.int_sway = 0.0
            self.int_yaw = 0.0
            self.prev_err_yaw = 0.0
            self.ref_surge = 0.0
            self.ref_sway = 0.0
            self.ref_yaw_rate = 0.0
            return # Avbryter funksjonen her. Publisere ingenting. Mode Manager styrer showet nå.

        # Er vi i Auto (2), kjører vanlig regulering:
        kp_surge = self.get_parameter('Kp_surge').value
        ki_surge = self.get_parameter('Ki_surge').value
        kp_sway = self.get_parameter('Kp_sway').value
        ki_sway = self.get_parameter('Ki_sway').value
        kp_yaw = self.get_parameter('Kp_yaw').value
        kd_yaw = self.get_parameter('Kd_yaw').value

        err_surge = self.ref_surge - self.act_surge
        self.int_surge += err_surge * self.dt
        tau_surge = (kp_surge * err_surge) + (ki_surge * self.int_surge)

        err_sway = self.ref_sway - self.act_sway
        self.int_sway += err_sway * self.dt
        tau_sway = (kp_sway * err_sway) + (ki_sway * self.int_sway)

        err_yaw = self.ref_yaw_rate - self.act_yaw_rate
        deriv_yaw = (err_yaw - self.prev_err_yaw) / self.dt
        tau_yaw = (kp_yaw * err_yaw) + (kd_yaw * deriv_yaw)
        self.prev_err_yaw = err_yaw

        msg = Wrench()
        msg.force.x = float(tau_surge)
        msg.force.y = float(tau_sway)
        msg.torque.z = float(tau_yaw)
        self.pub_thrust.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
