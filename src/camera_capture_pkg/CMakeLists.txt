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

# Add server executable
add_executable(service_member_function src/service_member_function.cpp)
ament_target_dependencies(service_member_function rclcpp sensor_msgs std_srvs std_msgs cv_bridge OpenCV)

# Add client executable
add_executable(client_member_function src/client_member_function.cpp)
ament_target_dependencies(client_member_function rclcpp std_srvs std_msgs)

# Add coordinate service node executable
add_executable(coordinate_service_node src/coordinate_service_node.cpp)
ament_target_dependencies(coordinate_service_node rclcpp std_srvs geometry_msgs yaml-cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(coordinate_service_node ${cpp_typesupport_target} ${YAML_CPP_LIBRARIES})

# Add behavior tree executor executable
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor rclcpp behaviortree_cpp_v3)

# Install targets
install(TARGETS
  service_member_function
  client_member_function
  coordinate_service_node
  bt_executor
  DESTINATION lib/${PROJECT_NAME})

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
