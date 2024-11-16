from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to the RViz2 config file
    rviz_config_path = "/home/nitzz/testingzedi2/ros2_cpp_ws/src/yolov8_ros2/rviz_config/zedi2PR.rviz"

    return LaunchDescription([
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
        
        # RViz2 node with specified config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],  # Load RViz2 config
            output='screen'
        ),
    ])

