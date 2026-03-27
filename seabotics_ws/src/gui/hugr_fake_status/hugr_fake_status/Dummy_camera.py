import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
from rclpy.qos import QoSProfile, HistoryPolicy



class DummyCameraNode(Node):
    """High-FPS dummy camera publishing frames with a moving object."""

    def __init__(self):
        super().__init__('dummy_camera')
        self.frame_rate = 30.0  # target FPS
        self.height = 480
        self.width = 640

        # Publisher with a larger queue to avoid dropped frames
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Image, '/stereo/right/image_raw', qos)
        self.bridge = CvBridge()

        # Precompute background gradient
        self.bg = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for i in range(3):  # BGR channels
            self.bg[:, :, i] = np.linspace(50, 200, self.width, dtype=np.uint8)

        # Object properties
        self.obj_radius = 30
        self.obj_color = (0, 0, 255)  # Red in BGR
        self.obj_x = 50
        self.obj_y = 100
        self.dx = 5
        self.dy = 3

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frame)

        self.get_logger().info(f"Dummy camera started, publishing at {self.frame_rate} FPS on /camera/image_raw")

    def generate_frame(self):
        """Generate a frame with a moving circle on precomputed background."""
        frame = self.bg.copy()

        # Draw moving circle
        y, x = np.ogrid[:self.height, :self.width]
        mask = (x - self.obj_x)**2 + (y - self.obj_y)**2 <= self.obj_radius**2
        for i in range(3):
            frame[:, :, i][mask] = self.obj_color[i]

        # Update position for next frame
        self.obj_x += self.dx
        self.obj_y += self.dy

        # Bounce off walls
        if self.obj_x - self.obj_radius < 0 or self.obj_x + self.obj_radius >= self.width:
            self.dx *= -1
        if self.obj_y - self.obj_radius < 0 or self.obj_y + self.obj_radius >= self.height:
            self.dy *= -1

        return frame

    def publish_frame(self):
        frame = self.generate_frame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()