cmake_minimum_required(VERSION 3.8)
project(output_rectified_image_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


ament_auto_add_executable(${PROJECT_NAME}_node
  src/output_rectified_image.cpp
  src/output_rectified_image_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES}) 

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)