
# YOLOv8 ROS2 Segmentation Project

This project integrates YOLOv8 segmentation with ROS2, enabling real-time object detection and segmentation using video input or live camera feeds. It also includes depth perception through a depth sensor.

## Setup Instructions

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-folder>
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

## Launch Instructions

### To use with a video sample:
```bash
ros2 launch yolov8_ros2 video_sample_launch.py
```

### To use with a live camera feed:
Modify the visualization blocks and set the topics to:
- Segmented image topic: `/camera/segmented_image`
- Raw camera feed: `/rgb/image_rect_color`

### Depth Sensor:
The depth sensor publishes data to the `/depth/depth_registered` topic.

## Dataset Creation Methodology

1. **Initial Bounding Boxes**:
   - Created bounding boxes using Grounding Dino.
   - Segmented images using SAM (Segment Anything Model by Meta).

2. **Data Augmentation**:
   - Adjusted parameters to refine results, selecting 300 high-quality images.
   - Augmented dataset with ±15° rotations, blur, and brightness variations, increasing the dataset to 1200 images.

3. **Model Training Iteration**:
   - Partially trained YOLOv8l on the augmented dataset to generate predictions for new images.
   - Enhanced the new dataset by applying further image processing (rotation, blur, brightness) while retaining annotation consistency.

4. **Final Training**:
   - Trained a fresh YOLOv8l model on the refined dataset to minimize overfitting.

## ONNX Model
The trained YOLOv8l model was converted to ONNX format. Find all models in the following directory:
```
zedi2/ros2_cpp_ws/src/yolov8_ros2/models
```

## Nodes Overview

### 1. **Video Publisher Node**
- Publishes frames from a video file to `/camera/image_raw`.
- Provides synchronized camera info to `/camera/camera_info`.

### 2. **YOLOv8 Segmentation Node**
- Subscribes to `/camera/image_raw`.
- Publishes segmented images to `/camera/segmented_image`.

### 3. **Depth Subscriber Node**
- Subscribes to `/camera/depth/image_raw`.
- Logs received depth image information.

### 4. **Launch Files**
- **`video_sample_launch.py`**: Launches all nodes with a sample video.
- **`startup_launch.py`**: Configures RViz and other nodes for testing.

---

Feel free to customize this further based on additional project requirements or details.
