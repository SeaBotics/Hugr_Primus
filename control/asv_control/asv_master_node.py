import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import serial

class ASVMasterNode(Node):
    def __init__(self):
        super().__init__('asv_master_node')
        
        # Seriell tilkobling til STM32
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("Seriell port åpnet.")
        except Exception as e:
            self.get_logger().error(f"Klarte ikke åpne seriellport: {e}")

        # Abonner på PlayStation-kontroller data
        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
            
        self.get_logger().info("ASV Master Node startet. Klar for input!")

    def joy_callback(self, joy_msg):
        # Mapping fra PS-kontroller
        surge = joy_msg.axes[1]   # Venstre stikke opp/ned
        sway  = joy_msg.axes[0]   # Venstre stikke venstre/høyre
        yaw   = joy_msg.axes[3]   # Høyre stikke venstre/høyre

        # Mixing for 4 thrustere (X-konfigurasjon)
        t1 = surge + sway + yaw  # Fremme Venstre
        t2 = surge - sway - yaw  # Fremme Høyre
        t3 = surge - sway + yaw  # Bak Venstre
        t4 = surge + sway - yaw  # Bak Høyre

        # Normalisering til verdier mellom -1.0 og 1.0
        speeds = [t1, t2, t3, t4]
        max_val = max([abs(x) for x in speeds] + [1.0])
        t1, t2, t3, t4 = [x / max_val for x in speeds]

        # Send pakken til STM32: "M:t1,t2,t3,t4\n"
        payload = f"M:{t1:.2f},{t2:.2f},{t3:.2f},{t4:.2f}\n"
        if hasattr(self, 'ser'):
            self.ser.write(payload.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ASVMasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()