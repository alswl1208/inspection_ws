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
  "srv/MoveToCoordinate.srv"
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# Add behavior tree executor executable
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor rclcpp behaviortree_cpp_v3)

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

add_executable(move_to_home_node src/move_to_home.cpp)
ament_target_dependencies(move_to_home_node
  rclcpp
  rclcpp_action
  nav2_msgs
)

# Install the executable
install(TARGETS
  move_to_coordinate_node
  move_to_home_node
  DESTINATION lib/${PROJECT_NAME})


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_coordinate_node ${cpp_typesupport_target} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES})

# Python 스크립트 설치
install(PROGRAMS
  camera_capture_pkg/camera_node.py  # 설치할 Python 스크립트
  camera_capture_pkg/coordinate_service_node.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories
install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
