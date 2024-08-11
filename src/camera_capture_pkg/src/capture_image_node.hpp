#pragma once

#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace BT;

class CaptureImageNode : public RosServiceNode<std_srvs::srv::Trigger>
{
public:
    CaptureImageNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params) {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setRequest(Request::SharedPtr& request) override;

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    NodeStatus onFailure(ServiceNodeErrorCode error) override;

    // onHalt를 halt로 변경합니다.
    void halt() override;
};
