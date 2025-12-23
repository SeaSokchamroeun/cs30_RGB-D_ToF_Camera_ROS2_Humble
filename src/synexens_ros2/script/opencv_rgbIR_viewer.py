#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class RGBOnlyViewer(Node):
    def __init__(self):
        super().__init__('rgb_only_viewer')
        self.rgb_img = None

        # Allow camera prefix to be overridden via parameter
        self.declare_parameter('camera_prefix', '/camera1_HV0130315L0317')
        prefix = self.get_parameter('camera_prefix').value

        # Subscribe only to RGB (no depth/IR)
        self.create_subscription(Image, f'{prefix}/rgb_raw', self.rgb_cb, 1)

        self.get_logger().info(f"RGB-only viewer started. Subscribed to: {prefix}/rgb_raw")
        self.get_logger().info("Press 'q' or ESC in the OpenCV window to quit.")

        # Create a resizable window (initial size half of 960x540)
        cv2.namedWindow("CS30 RGB", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CS30 RGB", 640, 360)

    def rgb_cb(self, msg):
        try:
            if msg.encoding == '8SC3':
                # Synexens raw signed char format
                img = np.frombuffer(msg.data, dtype=np.byte).reshape(
                    msg.height, msg.width, 3)
                # Convert to uint8 and ensure valid [0,255] range
                img = np.clip(img, 0, 255).astype(np.uint8)
                # Synexens uses RGB → OpenCV expects BGR
                self.rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                # Fallback: use cv_bridge for standard encodings (e.g., rgb8, bgr8)
                from cv_bridge import CvBridge
                bridge = CvBridge()
                self.rgb_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def display(self):
        if self.rgb_img is not None:
            cv2.imshow("CS30 RGB", self.rgb_img)
        # Required for OpenCV window to refresh
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # 'q' or ESC
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RGBOnlyViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # Low latency
            node.display()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()