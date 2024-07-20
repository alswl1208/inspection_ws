#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("capture_image_client");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
    node->create_client<std_srvs::srv::Trigger>("capture_image");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call succeeded: %s", result.get()->message.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service capture_image");
  }

  rclcpp::shutdown();
  return 0;
}
