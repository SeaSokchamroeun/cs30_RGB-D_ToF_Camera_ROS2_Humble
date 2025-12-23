#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OpenCVRgbViewer(Node):
    def __init__(self):
        super().__init__('opencv_rgb_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera1_HV0130315L0317/rgb_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info("Subscribed to RGB topic. Press 'q' in the OpenCV window to quit.")

    def listener_callback(self, msg):
        try:
            # Handle Synexens' non-standard '8SC3' encoding
            if msg.encoding == '8SC3':
                # Convert signed char (int8) to unsigned (uint8)
                img_array = np.frombuffer(msg.data, dtype=np.byte).reshape(
                    msg.height, msg.width, 3
                )
                cv_image = img_array.astype(np.uint8)
                # Optional: flip channels if BGR/RGB is swapped
                # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                # Standard ROS image (e.g., bgr8, rgb8)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display with OpenCV
            cv2.imshow("CS30 RGB (OpenCV)", cv_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 'q' or ESC
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVRgbViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
