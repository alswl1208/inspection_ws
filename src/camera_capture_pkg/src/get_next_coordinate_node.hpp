#ifndef GET_NEXT_COORDINATE_NODE_HPP
#define GET_NEXT_COORDINATE_NODE_HPP

#include "behaviortree_ros2/bt_service_node.hpp"
#include "camera_capture_pkg/srv/get_next_coordinate.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace BT;

class GetNextCoordinateNode : public RosServiceNode<camera_capture_pkg::srv::GetNextCoordinate>
{
public:
    GetNextCoordinateNode(const std::string& name, const NodeConfig& conf,
                          const RosNodeParams& params)
        : RosServiceNode<camera_capture_pkg::srv::GetNextCoordinate>(name, conf, params) {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({ OutputPort<geometry_msgs::msg::Point>("coordinate") });
    }

    bool setRequest(Request::SharedPtr& request) override;
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
    NodeStatus onFailure(ServiceNodeErrorCode error) override;
    
    // onHalt를 halt로 변경합니다.
    void halt() override;
};

#endif // GET_NEXT_COORDINATE_NODE_HPP
