from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    rviz_config_path = "/home/nitzz/testingzedi2/ros2_cpp_ws/src/yolov8_ros2/rviz_config/zedi2PR.rviz"
    bag_file_path = "/home/nitzz/PycharmProjects/zedzed/Zedi2-Segmentation-Tensorrt-Onnx/ros2_py_ws/src/yolov8_ros2/internship_assignment_sample_bag/internship_assignment_sample_bag_0.db3"

    return LaunchDescription([
        # Play ROS2 bag file
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file_path, '--loop'],
            output='screen'
        ),

        # YOLOv8 segmentation node
        Node(
            package='yolov8_ros2',
            executable='yolov8_segment_node',
            name='yolov8_segment_node'
        ),

        # Depth subscriber node
        Node(
            package='yolov8_ros2',
            executable='depth_subscriber_node',
            name='depth_subscriber_node'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])

