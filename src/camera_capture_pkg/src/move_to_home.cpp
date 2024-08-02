#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class MoveToHomeNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MoveToHomeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_home_node", options)
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Check if the action server is available
    while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    send_goal();
  }

private:
  void send_goal()
  {
    // Create a goal message with the home position
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal to home...");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MoveToHomeNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&MoveToHomeNode::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveToHomeNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server, waiting for result...");
    }
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation to home succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation to home was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation to home was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();  // Shut down the node after completing the navigation
  }

  void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToHomeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
