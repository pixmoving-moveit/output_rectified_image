import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
      package='output_rectified_image_cpp',
      executable='output_rectified_image_cpp_node',
      name='output_rectified_image_cpp_node',
      remappings=[
        ('image_raw', '/front/gmsl/image_raw'),
        ('camera_info', '/front/gmsl/camera_info'),

        ('rectified/image_raw', '/front/gmsl/rectified/image_raw'),
        ('rectified/camera_info', '/front/gmsl/rectified/camera_info')],
      output='screen')
    return LaunchDescription([node])