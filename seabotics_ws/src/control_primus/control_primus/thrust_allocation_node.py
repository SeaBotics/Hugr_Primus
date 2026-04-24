import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
import numpy as np

class ThrustAllocationNode(Node):
    def __init__(self):
        super().__init__('allocation_node')
        
        # Deklarer geometriske parametere
        self.declare_parameter('dy', 0.15) # Avstand CM til surge thrustere
        self.declare_parameter('lf', 0.345)  # Avstand Cm til baug thruster
        self.declare_parameter('lb', 0.35)  # Avstand CM til akter thruster

        self.subscription = self.create_subscription(Wrench, '/thrust_demand', self.demand_callback, 10)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/thruster_forces', 10)
        self.get_logger().info("Thrust Allocation med dynamisk geometri er klar.")

    def demand_callback(self, msg):
        # Hent nyeste geometri fra parametere
        dy = self.get_parameter('dy').value
        lf = self.get_parameter('lf').value
        lb = self.get_parameter('lb').value

        # Oppdater B-matrisen og finn pseudoinvers
        # Surge: T1+T2, Sway: T3+T4, Yaw: -T1*dy + T2*dy + T3*lf - T4*lb
        B = np.array([
            [1.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 1.0],
            [-dy, dy, lf, -lb]
        ])
        B_pinv = np.linalg.pinv(B)

        # Beregn krefter
        tau = np.array([msg.force.x, msg.force.y, msg.torque.z])
        f = np.dot(B_pinv, tau)

        # Publiser
        force_msg = Float64MultiArray()
        force_msg.data = f.tolist()
        self.publisher_.publish(force_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrustAllocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
