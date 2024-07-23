#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MoveToCoordinateNode : public rclcpp::Node
{
public:
  MoveToCoordinateNode()
  : Node("move_to_coordinate_node")
  {
    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    this->declare_parameter<double>("goal_x", 1.0);
    this->declare_parameter<double>("goal_y", 1.0);
    this->declare_parameter<double>("goal_z", 0.0);
    this->declare_parameter<double>("goal_w", 1.0);

    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MoveToCoordinateNode::send_goal, this));
  }

private:
  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    this->get_parameter("goal_x", goal_msg.pose.pose.position.x);
    this->get_parameter("goal_y", goal_msg.pose.pose.position.y);
    this->get_parameter("goal_z", goal_msg.pose.pose.position.z);
    this->get_parameter("goal_w", goal_msg.pose.pose.orientation.w);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MoveToCoordinateNode::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToCoordinateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
