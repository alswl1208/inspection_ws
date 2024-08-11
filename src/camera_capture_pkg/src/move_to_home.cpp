#include "move_to_home.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool MoveToHomeNode::setGoal(Goal& goal)
{
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (!goal_pose)
  {
    RCLCPP_ERROR(logger(), "Missing required input [goal_pose]");
    return false;
  }
  goal.pose = goal_pose.value();
  return true;
}

NodeStatus MoveToHomeNode::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "Navigation to home completed successfully.");
  return NodeStatus::SUCCESS;
}

NodeStatus MoveToHomeNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Navigation to home failed with error: %s", toStr(error));
  return NodeStatus::FAILURE;
}

void MoveToHomeNode::onHalt()
{
  RCLCPP_INFO(logger(), "Navigation to home halted.");
}

// Plugin registration.
CreateRosNodePlugin(MoveToHomeNode, "MoveToHomeNode");
