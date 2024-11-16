import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.bridge = CvBridge()

        # Path to your video file
        self.video_path = "/home/nitzz/Desktop/warehouse.mp4"
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open video file")
            return

        # Timer to publish frames at video frame rate
        self.timer_period = 1 / self.cap.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(self.timer_period, self.publish_frame)

        # Initialize CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_frame"
        self.camera_info_msg.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_info_msg.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Default intrinsic parameters for a pinhole camera model
        fx = 500.0  # Focal length in x
        fy = 500.0  # Focal length in y
        cx = self.camera_info_msg.width / 2.0  # Optical center x
        cy = self.camera_info_msg.height / 2.0  # Optical center y

        self.camera_info_msg.k = [fx, 0.0, cx,  # Intrinsic matrix
                                  0.0, fy, cy,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.p = [fx, 0.0, cx, 0.0,  # Projection matrix
                                  0.0, fy, cy, 0.0,
                                  0.0, 0.0, 1.0, 0.0]
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
        self.camera_info_msg.distortion_model = "plumb_bob"

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video or cannot read frame.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to reset video playback.")
                return

        # Convert OpenCV frame to ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.frame_id = "camera_frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish Image and CameraInfo messages
        self.publisher.publish(msg)
        self.camera_info_msg.header.stamp = msg.header.stamp  # Sync timestamps
        self.camera_info_pub.publish(self.camera_info_msg)

        self.get_logger().info("Published a frame with CameraInfo")

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

