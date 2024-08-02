#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "camera_capture_pkg/srv/get_next_coordinate.hpp"

class MoveToCoordinateNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GetNextCoordinate = camera_capture_pkg::srv::GetNextCoordinate;

  explicit MoveToCoordinateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_coordinate_node", options), current_index_(0)
  {
    client_ = this->create_client<GetNextCoordinate>("get_next_coordinate");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    send_goal();
  }

private:
  void send_goal()
  {
    auto request = std::make_shared<GetNextCoordinate::Request>();
    request->coordinate_name = "position" + std::to_string(current_index_ + 1);

    using ServiceResponseFuture = rclcpp::Client<GetNextCoordinate>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      if (response) {
        this->navigate_to_position(response);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get coordinate from service");
      }
    };

    auto result = client_->async_send_request(request, response_received_callback);
  }

  void navigate_to_position(std::shared_ptr<GetNextCoordinate::Response> position)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = position->x;
    goal_msg.pose.pose.position.y = position->y;
    goal_msg.pose.pose.position.z = position->z;
    goal_msg.pose.pose.orientation.x = position->orientation_x;
    goal_msg.pose.pose.orientation.y = position->orientation_y;
    goal_msg.pose.pose.orientation.z = position->orientation_z;
    goal_msg.pose.pose.orientation.w = position->orientation_w;

    RCLCPP_INFO(this->get_logger(), "Sending goal request...");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveToCoordinateNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&MoveToCoordinateNode::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveToCoordinateNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    current_index_++;
    if (current_index_ < total_positions_) {
      RCLCPP_INFO(this->get_logger(), "Moving to next coordinate: position%d", current_index_ + 1);
      send_goal();
    } else {
      RCLCPP_INFO(this->get_logger(), "All coordinates processed. Shutting down.");
      rclcpp::shutdown();
    }
  }

  void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
  }

  rclcpp::Client<GetNextCoordinate>::SharedPtr client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  size_t current_index_;
  const size_t total_positions_ = 2; // YAML 파일의 총 포지션 수를 설정합니다. 필요에 따라 변경하세요.
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToCoordinateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
