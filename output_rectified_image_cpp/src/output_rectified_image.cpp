#include <output_rectified_image/output_rectified_image.hpp>

#include <iostream>
#include <chrono>

OutputRectifiedImage::OutputRectifiedImage(/* args */) : Node("output_rectified_image_cpp_node"),
m_rect_image_msg(new sensor_msgs::msg::Image()),
m_rect_camera_info_msg(new sensor_msgs::msg::CameraInfo())
{
  // 订阅原始图像和相机信息的话题
  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(), std::bind(&OutputRectifiedImage::imageCallback, this, std::placeholders::_1));

  camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10, std::bind(&OutputRectifiedImage::cameraInfoCallback, this, std::placeholders::_1));

  // 创建发布去畸变图像和相机信息的话题
  rectified_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rectified/image_raw", 10);
  rectified_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rectified/camera_info", 10);

}

OutputRectifiedImage::~OutputRectifiedImage(/* args */)
{
  m_rect_camera_info_msg.reset();
}

void OutputRectifiedImage::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  original_camera_info_ = *msg;
}

void OutputRectifiedImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 将ROS图像消息转换为OpenCV格式
  image_raw_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  
  {
    // 获取内参和畸变
    if(is_get_camera_info_){
      cv::Mat camera_matrix;
      cv::Mat distortion_coefficients;

      camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(original_camera_info_.k.data()));
      distortion_coefficients = cv::Mat(1, original_camera_info_.d.size(), CV_64F, const_cast<double *>(original_camera_info_.d.data()));

      cv::Size size(original_camera_info_.width, original_camera_info_.height);
      cv::Size resize(original_camera_info_.width/image_resize_, original_camera_info_.height/image_resize_);

      cv::Mat new_K = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, size, 0, resize);

      cv::initUndistortRectifyMap(
        camera_matrix, distortion_coefficients, cv::Mat(), 
        new_K, resize, CV_32FC1, undistort_map_x_, undistort_map_y_);

      *m_rect_camera_info_msg = original_camera_info_;
      m_rect_camera_info_msg->d.resize(5); 
      std::array<double, 12UL> p = m_rect_camera_info_msg->p;
      p[0] =  m_rect_camera_info_msg->k[0];
      p[2] =  m_rect_camera_info_msg->k[2];
      p[5] =  m_rect_camera_info_msg->k[4];
      p[6] =  m_rect_camera_info_msg->k[5];
      m_rect_camera_info_msg->p = p;
      
      is_get_camera_info_ = false;
    }

    cv::remap(image_raw_, image_rect_, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);
  }
  
  // 将OpenCV格式的图像转换为ROS消息
  m_rect_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), msg->encoding, image_rect_).toImageMsg();
  m_rect_image_msg->header = msg->header;
  m_rect_camera_info_msg->header = msg->header;
  // 发布去畸变图像
  rectified_image_publisher_->publish(*m_rect_image_msg);
  rectified_camera_info_publisher_->publish(*m_rect_camera_info_msg);
}
