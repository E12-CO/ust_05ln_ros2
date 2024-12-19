cmake_minimum_required(VERSION 3.8)
project(ust_05ln_ros2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

find_package(ament_cmake REQUIRED)
# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ust_dependencies
	rclcpp
	rclcpp_components
	sensor_msgs
)

add_executable(urg_node
	src/ust_05ln_ros2.cpp
)

target_link_libraries(
	urg_node
)

ament_target_dependencies(
	urg_node
	${ust_dependencies}
)

install(
	TARGETS urg_node
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY params launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
