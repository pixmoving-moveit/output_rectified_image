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
  
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

public:
  OutputRectifiedImage(/* args */);
  ~OutputRectifiedImage(/* args */);
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectified_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rectified_camera_info_publisher_;

  sensor_msgs::msg::Image::SharedPtr m_rect_image_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr m_rect_camera_info_msg;

  bool is_get_camera_info_=true;

  cv::Mat image_raw_;
  cv::Mat image_rect_;

  cv::Mat undistort_map_x_, undistort_map_y_;
  uint8_t image_resize_=1;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  sensor_msgs::msg::CameraInfo original_camera_info_;
};

#endif // output_rectified_image_cpp_HPP_