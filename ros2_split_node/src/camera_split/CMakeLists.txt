cmake_minimum_required(VERSION 3.5)
project(camera_split)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

#find_package(OpenCV REQUIRED)
find_package(OpenCV 3.2.0 QUIET
	PATHS /usr/local/opencv/opencv3.2.0
	NO_DEFAULT_PATH
)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_library(camera_split_lib
   src/camera_split.cpp)
target_link_libraries(camera_split_lib  ${OpenCV_LIBS})

ament_target_dependencies(camera_split_lib rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_info_manager image_transport)

add_executable(camera_split_node src/camera_split.cpp)
ament_target_dependencies(camera_split_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_info_manager image_transport)
target_link_libraries(camera_split_node camera_split_lib) 


# added from DEBUG

# Install launch files

# Install nodes
install(
  TARGETS camera_split_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

