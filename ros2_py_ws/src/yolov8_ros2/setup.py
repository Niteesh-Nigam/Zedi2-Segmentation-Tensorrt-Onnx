from setuptools import setup, find_packages

package_name = 'yolov8_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['src']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/startup_launch.py', 
            'launch/video_sample_launch.py'
        ]),
        ('share/' + package_name + '/rviz_config', [
        'rviz_config/zedi2PR.rviz'
    	]),
    	('share/' + package_name + '/internship_assignment_sample_bag', [
        'internship_assignment_sample_bag/internship_assignment_sample_bag_0.db3'
    	]),
        ('share/' + package_name + '/test videos', ['test videos/warehouse.mp4']),
        ('share/' + package_name + '/test videos', ['test videos/warehouse1.mp4']),
        ('share/' + package_name + '/models', [
            #'models/last.pt', 
            'models/best.pt',
            #'models/last1.pt', 
            #'models/last2.pt', 
            #'models/best1.pt',
            'models/best3.pt', 
           # 'models/best2.pt', 
        ]),
        ('share/' + package_name + '/internship_assignment_sample_bag', [
            'internship_assignment_sample_bag/internship_assignment_sample_bag_0.db3'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nitzz',
    maintainer_email='niteesh.nigam99@gmail.com',
    description='ROS2 package for YOLOv8-based segmentation and processing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_segment_node = src.yolov8_segment_node:main',
            'depth_subscriber_node = src.depth_subscriber_node:main',
            'video_publisher_node = src.video_publisher_node:main',
        ],
    },
)

