#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_executor");
  RCLCPP_INFO(node->get_logger(), "BT Executor node has been started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
