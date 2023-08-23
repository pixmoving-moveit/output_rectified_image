#!/usr/bin/env python3
# coding=utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
class UndistortedImageNode(Node):
    def __init__(self):
        super().__init__('undistorted_image_node')
        # Pub
        self.undistorted_image_publisher = self.create_publisher(
            Image,
            "output_image_raw",
            10)
        self.undistorted_camera_info_publisher = self.create_publisher(
            CameraInfo,
            "output_camera_info",
            10)

        # Sub
        self.subscription = self.create_subscription(
            Image,
            "input_image_raw",
            self.image_callback,
            qos_profile_sensor_data)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "input_camera_info",
            self.camera_info_callback,
            10)
        # 
        self.cv_bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None

    def camera_info_callback(self, msg:CameraInfo):
        numpy_array = np.array(msg.k)
        self.camera_matrix =  numpy_array.reshape(3, 3)
        self.distortion_coefficients = np.array(msg.d)
    
        # 获取优化的相机投影矩阵
        # self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        #     self.camera_matrix, self.distortion_coefficients,
        #     (msg.width, msg.height), 1, (msg.width, msg.height))
        self.p =  msg.p
        self.frame = msg.header.frame_id

    def image_callback(self, msg):
        if self.camera_matrix is None or self.distortion_coefficients is None:
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        undistorted_image = cv2.undistort(
            cv_image, self.camera_matrix, self.distortion_coefficients)
            # None, self.new_camera_matrix)
        
        # 进行后续处理，例如发布去畸变图像
        undistorted_msg = self.cv_bridge.cv2_to_imgmsg(undistorted_image, 'bgr8')
        undistorted_msg.header.frame_id =  self.frame
        undistorted_msg.header.stamp = self.get_clock().now().to_msg()
        self.undistorted_image_publisher.publish(undistorted_msg)

        # 更新相机信息的内参矩阵和畸变系数
        undistorted_camera_info = CameraInfo()
        undistorted_camera_info.header = msg.header
        undistorted_camera_info.height = undistorted_image.shape[0]
        undistorted_camera_info.width = undistorted_image.shape[1]
        undistorted_camera_info.k = self.camera_matrix.ravel().tolist()
        undistorted_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        undistorted_camera_info.p = self.p # 假设不进行投影变换
        undistorted_camera_info.distortion_model = 'plumb_bob'

        self.undistorted_image_publisher.publish(undistorted_msg)
        self.undistorted_camera_info_publisher.publish(undistorted_camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = UndistortedImageNode()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
