
#include <output_rectified_image/output_rectified_image.hpp>


int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OutputRectifiedImage>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}