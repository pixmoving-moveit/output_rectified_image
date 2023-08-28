#ifndef output_rectified_image_cpp_HPP_
#define output_rectified_image_cpp_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


class OutputRectifiedImage : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectified_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rectified_camera_info_publisher_;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  sensor_msgs::msg::CameraInfo original_camera_info_;
public:
  OutputRectifiedImage(/* args */);
};

#endif // output_rectified_image_cpp_HPP_