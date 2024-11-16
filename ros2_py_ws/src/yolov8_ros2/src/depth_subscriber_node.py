import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DepthSubscriberNode(Node):
    def __init__(self):
        super().__init__('depth_subscriber_node')
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)

    def depth_callback(self, msg):
        self.get_logger().info(f"Received depth image: {msg.width}x{msg.height}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

