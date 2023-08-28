#include <output_rectified_image/output_rectified_image.hpp>

#include <iostream>
#include <chrono>

OutputRectifiedImage::OutputRectifiedImage(/* args */) : Node("output_rectified_image_cpp_node")
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

void OutputRectifiedImage::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // 保存相机矩阵和畸变系数
  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data()));
  distortion_coefficients_ = cv::Mat(1, msg->d.size(), CV_64F, const_cast<double *>(msg->d.data()));

  original_camera_info_ = *msg;

}

void OutputRectifiedImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // 将ROS图像消息转换为OpenCV格式
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // 进行去畸变操作
  cv::Mat rectified_image;
  cv::undistort(cv_ptr->image, rectified_image, camera_matrix_, distortion_coefficients_);

  // 将OpenCV格式的图像转换为ROS消息
  sensor_msgs::msg::Image::SharedPtr rectified_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified_image).toImageMsg();
  rectified_msg->header = msg->header;
  auto image_ptr = std::make_unique<sensor_msgs::msg::Image>(*rectified_msg);
  // 发布去畸变图像
  rectified_image_publisher_->publish(std::move(image_ptr));


  original_camera_info_.header = msg->header;
  original_camera_info_.d = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 12UL> p = original_camera_info_.p;
  p[0] =  original_camera_info_.k[0];
  p[2] =  original_camera_info_.k[2];
  p[5] =  original_camera_info_.k[4];
  p[6] =  original_camera_info_.k[5];
  original_camera_info_.p = p;
  // 发布去畸变相机信息
  rectified_camera_info_publisher_->publish(original_camera_info_);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "程序运行时间: " << duration.count() << " 毫秒" << std::endl;

}
