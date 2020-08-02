cmake_minimum_required(VERSION 3.5)
project(pwm_pca9685)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-li2c)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(mixer_node src/mixer_node.cpp)
ament_target_dependencies(mixer_node rclcpp std_msgs geometry_msgs)

add_executable(pca9685_node src/pca9685_node.cpp)
ament_target_dependencies(pca9685_node rclcpp std_msgs)

install(TARGETS pca9685_node mixer_node
DESTINATION lib/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS)
