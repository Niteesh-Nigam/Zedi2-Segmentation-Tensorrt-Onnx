import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class YOLOv8SegmentPublisherNode(Node):
    def __init__(self):
        super().__init__('yolov8_segment_publisher_node')
        self.bridge = CvBridge()

        # Load YOLOv8 model
        model_path = "/home/nitzz/testingzedi2/ros2_cpp_ws/src/yolov8_ros2/last.pt"  # Replace with your model path
        self.model = YOLO(model_path)

        # Subscribe to the video stream
        self.video_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Input video topic
            self.image_callback,
            10
        )

        # Publisher for segmented images
        self.segmented_image_pub = self.create_publisher(Image, '/camera/segmented_image', 10)

        self.get_logger().info("YOLOv8 Segment Publisher Node Initialized")

    def image_callback(self, msg):
        """Callback function to process video frames and publish segmented images."""
        try:
            # Convert ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform inference
            results = self.model.predict(source=frame, conf=0.3, save=False)

            # Annotate the frame with segmentation results
            #annotated_frame = self.annotate_image(frame, results)
            annotated_frame = results[0].plot()

            # Convert the annotated frame to ROS2 Image message
            segmented_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            segmented_msg.header.frame_id = "camera_frame"
            segmented_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish the segmented image
            self.segmented_image_pub.publish(segmented_msg)
            self.get_logger().info("Published segmented image.")
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def annotate_image(self, frame, results):
        """
        Draw bounding boxes and segmentation masks on the original image.
        """
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # Bounding boxes
            scores = result.boxes.conf.cpu().numpy()  # Confidence scores
            class_ids = result.boxes.cls.cpu().numpy()  # Class IDs
            masks = result.masks.data.cpu().numpy()  # Segmentation masks

            for box, score, class_id, mask in zip(boxes, scores, class_ids, masks):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"Class {int(class_id)}: {score:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Overlay the mask on the image
                mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                colored_mask = np.zeros_like(frame)
                colored_mask[:, :, 1] = (mask * 255).astype(np.uint8)  # Green mask
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

