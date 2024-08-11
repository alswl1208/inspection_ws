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
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

# Service, Action, and Message files generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImage.srv"
  "srv/GetNextCoordinate.srv"
  "action/MoveToCoordinate.action"
  "action/MoveToHome.action"
  "action/ExecuteTree.action"
  "msg/NodeStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# 추가: 생성된 인터페이스를 빌드 시스템에 알려주기 위해 다음을 추가합니다.
ament_export_dependencies(rosidl_default_runtime)

# Add move to coordinate node library
add_library(move_to_coordinate_node SHARED src/move_to_coordinate_node.cpp)
ament_target_dependencies(move_to_coordinate_node
  rclcpp
  nav2_msgs
  rclcpp_action
  geometry_msgs
  yaml-cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  behaviortree_ros2
)
target_include_directories(move_to_coordinate_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(move_to_coordinate_node
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add move to home node library
add_library(move_to_home_node SHARED src/move_to_home.cpp)
ament_target_dependencies(move_to_home_node
  rclcpp
  rclcpp_action
  nav2_msgs
  behaviortree_cpp_v3
  behaviortree_ros2
)
target_include_directories(move_to_home_node PUBLIC
  ${behaviortree_ros2_INCLUDE_DIRS}
)
target_link_libraries(move_to_home_node
  ${behaviortree_ros2_LIBRARIES}
)

# Add capture image node library
add_library(capture_image_node SHARED src/capture_image_node.cpp)
ament_target_dependencies(capture_image_node
  rclcpp
  std_srvs
  sensor_msgs
  cv_bridge
  tf2_ros
  behaviortree_cpp_v3
  behaviortree_ros2
)
target_include_directories(capture_image_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(capture_image_node
  ${OpenCV_LIBRARIES}
)

# Add get next coordinate node library
add_library(get_next_coordinate_node SHARED src/get_next_coordinate_node.cpp)
ament_target_dependencies(get_next_coordinate_node
  rclcpp
  std_srvs
  behaviortree_cpp_v3
  behaviortree_ros2
  tf2_ros
)

# Add BT executor executable
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp_v3
  behaviortree_ros2
  std_msgs
)
target_link_libraries(bt_executor
  move_to_coordinate_node
  move_to_home_node
  capture_image_node
  get_next_coordinate_node
)

# Install the executables
install(TARGETS
  move_to_coordinate_node
  move_to_home_node
  capture_image_node
  get_next_coordinate_node
  bt_executor
  DESTINATION lib/${PROJECT_NAME})

# Link generated service/action/message to the nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(move_to_coordinate_node ${cpp_typesupport_target})
target_link_libraries(move_to_home_node ${cpp_typesupport_target})
target_link_libraries(capture_image_node ${cpp_typesupport_target})
target_link_libraries(get_next_coordinate_node ${cpp_typesupport_target})
target_link_libraries(bt_executor ${cpp_typesupport_target})

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
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
