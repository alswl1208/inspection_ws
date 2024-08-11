#include "move_to_coordinate_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


bool MoveToCoordinateNode::setGoal(Goal &goal)
{
    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    auto theta = getInput<double>("theta");

    if (!x || !y || !theta)
    {
        RCLCPP_ERROR(logger(), "Missing input");
        return false;
    }

    goal.pose.pose.position.x = x.value();
    goal.pose.pose.position.y = y.value();
    goal.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, theta.value(), 1.0));

    return true;
}

NodeStatus MoveToCoordinateNode::onResultReceived(const WrappedResult &wr)
{
    RCLCPP_INFO(logger(), "Goal reached successfully");
    return NodeStatus::SUCCESS;
}

NodeStatus MoveToCoordinateNode::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Goal failed with error: %s", toStr(error));
    return NodeStatus::FAILURE;
}

void MoveToCoordinateNode::onHalt()
{
    RCLCPP_INFO(logger(), "Action halted");
}

// Plugin registration
CreateRosNodePlugin(MoveToCoordinateNode, "MoveToCoordinate");

#ifndef MOVE_TO_COORDINATE_NODE_HPP
#define MOVE_TO_COORDINATE_NODE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;

class MoveToCoordinateNode : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    MoveToCoordinateNode(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("theta")
        });
    }

    bool setGoal(Goal &goal) override;
    void onHalt() override;
    NodeStatus onResultReceived(const WrappedResult &wr) override;
    NodeStatus onFailure(ActionNodeErrorCode error) override;
};

#endif // MOVE_TO_COORDINATE_NODE_HPP
