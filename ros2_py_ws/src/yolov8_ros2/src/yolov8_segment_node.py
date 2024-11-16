import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from ultralytics import YOLO
import os


class YOLOv8SegmentPublisherNode(Node):
    def __init__(self):
        super().__init__('yolov8_segment_publisher_node')
        self.bridge = CvBridge()

        # Dynamically locate the model path
        try:
            package_share_dir = get_package_share_directory('yolov8_ros2')
            model_path = os.path.join(package_share_dir, 'models', 'last.pt')
        except KeyError:
            self.get_logger().error("Could not find package 'yolov8_ros2'. Ensure it is installed and sourced.")
            return

        self.get_logger().info(f"Model path: {model_path}")

        # Load the YOLO model
        self.model = YOLO(model_path)

        # subscriber to video frsmes
        self.video_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.segmented_image_pub = self.create_publisher(Image, '/camera/segmented_image', 10)

        self.get_logger().info("YOLOv8 Segment Publisher Node Initialized")

    def image_callback(self, msg):
        """Callback function to process video frames and publish segmented images."""
        try:
        
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform inference
            results = self.model.predict(source=frame, conf=0.3, save=False)
            annotated_frame = results[0].plot()

            #annotated frame to ROS2 image message
            segmented_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            segmented_msg.header.frame_id = "camera_frame"
            segmented_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish segmented image
            self.segmented_image_pub.publish(segmented_msg)
            self.get_logger().info("Published segmented image.")
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def annotate_image(self, frame, results):
        """
        Draw bounding boxes and segmentation masks on the original image.
        """
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy()
            masks = result.masks.data.cpu().numpy()

            for box, score, class_id, mask in zip(boxes, scores, class_ids, masks):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"Class {int(class_id)}: {score:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # overlay the mask on the image
                mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                colored_mask = np.zeros_like(frame)
                colored_mask[:, :, 1] = (mask * 255).astype(np.uint8)
                frame = cv2.addWeighted(frame, 1, colored_mask, 0.5, 0)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8SegmentPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

