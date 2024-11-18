#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class DepthSubscriberNode(Node):
    def __init__(self):
        super().__init__('depth_subscriber_node')
        self.bridge = CvBridge()

        # Subscription to depth topic
        self.depth_subscriber = self.create_subscription(
            Image,
            '/depth/depth_registered',
            self.depth_callback,
            10
        )

        self.get_logger().info("Depth Subscriber Node Initialized.")

    def depth_callback(self, msg):
        """Callback function to process depth frames."""
        try:
            # Convert ROS2 Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Normalize depth values for visualization
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # Display the depth image
            cv2.imshow("Depth Image", depth_colored)
            cv2.waitKey(1)

            self.get_logger().info("Processed a depth frame.")
        except Exception as e:
            self.get_logger().error(f"Error processing depth frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

