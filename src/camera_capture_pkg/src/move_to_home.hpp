#ifndef MOVE_TO_HOME_HPP
#define MOVE_TO_HOME_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;

class MoveToHomeNode : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MoveToHomeNode(const std::string& name, const NodeConfig& conf,
                 const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ InputPort<geometry_msgs::msg::PoseStamped>("goal_pose") });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

#endif // MOVE_TO_HOME_HPP
