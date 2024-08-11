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
