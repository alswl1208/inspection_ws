cmake_minimum_required(VERSION 3.8)
project(camera_capture_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
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

# Define a set of dependencies for reuse
set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  geometry_msgs
  nav_msgs
  std_srvs
  yaml-cpp
  nav2_msgs
  behaviortree_cpp_v3
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_ros2
)

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

# Export the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# Add BT executor executable
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor ${THIS_PACKAGE_DEPS})
target_link_libraries(bt_executor
  bt_move_to_coordinate_plugin
  bt_move_to_home_plugin
  bt_capture_image_plugin
  bt_get_next_coordinate_plugin
)

# Add move to coordinate node library
add_library(bt_move_to_coordinate_plugin SHARED src/move_to_coordinate_node.cpp)
target_compile_definitions(bt_move_to_coordinate_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bt_move_to_coordinate_plugin ${THIS_PACKAGE_DEPS})
target_include_directories(bt_move_to_coordinate_plugin PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(bt_move_to_coordinate_plugin
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add move to home node library
add_library(bt_move_to_home_plugin SHARED src/move_to_home.cpp)
target_compile_definitions(bt_move_to_home_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bt_move_to_home_plugin ${THIS_PACKAGE_DEPS})
target_include_directories(bt_move_to_home_plugin PUBLIC
  ${behaviortree_ros2_INCLUDE_DIRS}
)
target_link_libraries(bt_move_to_home_plugin
  ${behaviortree_ros2_LIBRARIES}
)

# Add capture image node library
add_library(bt_capture_image_plugin SHARED src/capture_image_node.cpp)
target_compile_definitions(bt_capture_image_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bt_capture_image_plugin ${THIS_PACKAGE_DEPS})
target_include_directories(bt_capture_image_plugin PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bt_capture_image_plugin
  ${OpenCV_LIBRARIES}
)

# Add get next coordinate node library
add_library(bt_get_next_coordinate_plugin SHARED src/get_next_coordinate_node.cpp)
target_compile_definitions(bt_get_next_coordinate_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(bt_get_next_coordinate_plugin ${THIS_PACKAGE_DEPS})

# Install the executables and plugins
install(TARGETS
  bt_move_to_coordinate_plugin
  bt_move_to_home_plugin
  bt_capture_image_plugin
  bt_get_next_coordinate_plugin
  bt_executor
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

# Link generated service/action/message to the nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bt_move_to_coordinate_plugin ${cpp_typesupport_target})
target_link_libraries(bt_move_to_home_plugin ${cpp_typesupport_target})
target_link_libraries(bt_capture_image_plugin ${cpp_typesupport_target})
target_link_libraries(bt_get_next_coordinate_plugin ${cpp_typesupport_target})
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

ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()
