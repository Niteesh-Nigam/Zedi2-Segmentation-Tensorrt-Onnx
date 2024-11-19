
# YOLOv8 ROS2 Segmentation Project

This project integrates YOLOv8 segmentation with ROS2, enabling real-time object detection and segmentation using ROs2-Bag, video input or live camera feeds.

Below you will find the steps to configure and launch using a .db3 bag file, mp4 and steps to launch with live camera feed(Zed Wrapper not included)

Skip to find out Dataset Creation Ideology and Steps.

# check out how to configure and run launch file in this video
https://drive.google.com/file/d/1jNNZgYGnExfzsVlry06uCLTBNI4Y0QEc/view?usp=sharing

or follow the steps below

Models are located in ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/models/
(best.pt, best1.pt, best2_dynamic.onnx, best21920x1080dynamic.engine)

onnx and tensorrt have been set to dynamic for varying image size as Zedi2 produces configurable resolutions)

make sure to set the configuration score in yolov8_segment_node.py (Line 59)
located at ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/src/

## Setup Instructions
### To use with a ROS2-BAG (.db3) file.

1. Clone the repository:
   ```bash
   git clone https://github.com/Niteesh-Nigam/Zedi2-Segmentation-Tensorrt-Onnx.git
   cd Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Replace the ros2 bag file or use the existing one.
   ```bash
   cd yolov8_ros2/internship_assignment_sample_bag
   *replace bag db3 and .yaml configuration file - This file is the configuration file of your camera eg. Zedx or Zedi2 or the configurations from your last recording* 
   ```
4. Update the launch file *startup_launch.py* and *setup.py* with your new ros2-bag files.
   These are located in the directories
   ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/launch   (line 9)
   ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/   (line 26-27)
   respectively.
   (Check the video link for detailed tutorial)   

5. Build the workspace:
   ```bash
   cd ../.. (you should be in the ros2_ws directory)
   colcon build
   ```

### Launch Instructions
   ```bash
   source install/setup.bash
   ros2 launch yolov8_ros2 startup_launch.py 
   ```

### To use with a video sample:
*Follow steps 1 and 2 from the previous section*
3. Replace the .mp4 file or use the existing one.
   ```bash
   cd yolov8_ros2/test_videos
   *replace mp4 files*

4. Update the video publisher node *video_publisher_node.py* and *setup.py* with your new mp4 ve files.
   These are located in the directories
   ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/src/  (line 21)
   ../Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/   (line 16-17)
   respectively.
   (Check the video link for detailed tutorial)

5. Build the workspace:
   ```bash
   cd ../.. (you should be in the ros2_ws directory)
   colcon build
   ```
### Launch Instructions
   ```bash
   source install/setup.bash
   ros2 launch yolov8_ros2 video_sample_launch.py 
   ```

### To use with a live camera feed:

1. Ensure that you have Zed-Wrapped(Not Included in repository) with a minimum of Cuda 12.0
2. launch camera using the wrapper included files,making sure to use to correct configurations (Zedx.yaml, Zedi2.yaml etc.)
3. Modify the Rviz2 visualization blocks and set the topics to:
   - Segmented image topic: `/camera/segmented_image`
   - Raw camera feed: `/robot1/zed2i/left/image_rect_color`

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
- Publishes frames from a video file to `/robot1/zed2i/left/image_rect_color`.
- Provides synchronized camera info to `/camera/camera_info`.

### 2. **YOLOv8 Segmentation Node**
- Subscribes to `/camera/image_raw`.
- Publishes segmented images to `/camera/segmented_image`.

### 3. **Depth Subscriber Node**
- Subscribes to `/depth/depth_registered`.
- Logs received depth image information.

### 4. **Launch Files**
- **`video_sample_launch.py`**: Launches all nodes with a sample video.
- **`startup_launch.py`**: Configures RViz and other nodes for testing.

---

Feel free to customize this further based on additional project requirements or details.
