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
  // 获取内参和畸变
  if(is_get_camera_info_){
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;

    camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data()));
    distortion_coefficients = cv::Mat(1, msg->d.size(), CV_64F, const_cast<double *>(msg->d.data()));

    cv::Size size(msg->width, msg->height);
    cv::Size resize(msg->width/image_resize_, msg->height/image_resize_);

    cv::Mat new_K = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, size, 0, resize);

    cv::initUndistortRectifyMap(
      camera_matrix, distortion_coefficients, cv::Mat(), 
      new_K, resize, CV_32FC1, undistort_map_x_, undistort_map_y_);

    *m_rect_camera_info_msg = *msg;
    m_rect_camera_info_msg->d.resize(5); 
    std::array<double, 12UL> p = m_rect_camera_info_msg->p;
    p[0] =  m_rect_camera_info_msg->k[0];
    p[2] =  m_rect_camera_info_msg->k[2];
    p[5] =  m_rect_camera_info_msg->k[4];
    p[6] =  m_rect_camera_info_msg->k[5];
    m_rect_camera_info_msg->p = p;
    
    is_get_camera_info_ = false;
  }
}

void OutputRectifiedImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 将ROS图像消息转换为OpenCV格式
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    image_raw_ =  cv_ptr->image.clone();
   } catch (cv_bridge::Exception& e) {
    // 错误处理
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }
  if(!is_get_camera_info_){
    cv::remap(image_raw_, image_rect_, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);
    // 将OpenCV格式的图像转换为ROS消息
    m_rect_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), msg->encoding, image_rect_).toImageMsg();
    m_rect_image_msg->header = msg->header;
    m_rect_camera_info_msg->header = msg->header;
    // 发布去畸变图像
    rectified_image_publisher_->publish(*m_rect_image_msg);
    rectified_camera_info_publisher_->publish(*m_rect_camera_info_msg);
  }else{
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "没有获取camera_info");
  }
}
