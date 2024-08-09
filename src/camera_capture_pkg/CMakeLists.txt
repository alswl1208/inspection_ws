cmake_minimum_required(VERSION 3.8)
project(camera_capture_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImage.srv"
  "srv/GetNextCoordinate.srv"
  "action/MoveToCoordinate.action"
  "action/MoveToHome.action"
  "action/ExecuteTree.action"
  "msg/NodeStatus.msg"
  DEPENDENCIES builtin_interfaces
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Add move to coordinate node executable
add_executable(move_to_coordinate_node src/move_to_coordinate_node.cpp)
ament_target_dependencies(move_to_coordinate_node
  rclcpp
  nav2_msgs
  rclcpp_action
  geometry_msgs
  behaviortree_cpp_v3
  yaml-cpp
  tf2
  tf2_ros
)
target_link_libraries(move_to_coordinate_node
  ${behaviortree_cpp_v3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(move_to_home_node src/move_to_home.cpp)
ament_target_dependencies(move_to_home_node
  rclcpp
  rclcpp_action
  nav2_msgs
)

# Install the executables
install(TARGETS
  move_to_coordinate_node
  move_to_home_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_coordinate_node ${cpp_typesupport_target})

# Python scripts installation
install(PROGRAMS
  camera_capture_pkg/camera_node.py
  camera_capture_pkg/coordinate_service_node.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories
install(DIRECTORY
  behavior_trees
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
