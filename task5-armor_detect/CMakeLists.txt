cmake_minimum_required(VERSION 3.8)
project(armor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Torch_DIR /home/wtz/libtorch/share/cmake/Torch)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED) 
find_package(Torch REQUIRED)

add_executable(armor_detect src/armor_detect.cpp)

ament_target_dependencies(armor_detect
  rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)
target_link_libraries(armor_detect
  CURL::libcurl 
)
target_link_libraries(armor_detect
  ${TORCH_LIBRARIES}
)

install(TARGETS armor_detect
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY checkpoint/
        DESTINATION share/${PROJECT_NAME}/checkpoint)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
