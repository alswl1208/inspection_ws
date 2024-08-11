#include "get_next_coordinate_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool GetNextCoordinateNode::setRequest(Request::SharedPtr& request)
{
    // request를 설정하는 로직 구현
    return true; // 실제 로직에 따라 이 값을 수정하세요
}

NodeStatus GetNextCoordinateNode::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->success)
    {
        geometry_msgs::msg::Point coordinate;
        coordinate.x = response->x;
        coordinate.y = response->y;
        coordinate.z = response->z;

        setOutput("coordinate", coordinate);
        return NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(logger(), "Failed to get next coordinate: %s", response->message.c_str());
        return NodeStatus::FAILURE;
    }
}

NodeStatus GetNextCoordinateNode::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Service failed with error: %s", toStr(error));
    return NodeStatus::FAILURE;
}

// halt 메서드는 한 번만 정의해야 합니다.
void GetNextCoordinateNode::halt()
{
    RCLCPP_INFO(logger(), "Service halted.");
    BT::RosServiceNode<camera_capture_pkg::srv::GetNextCoordinate>::halt();
}

// Plugin registration
CreateRosNodePlugin(GetNextCoordinateNode, "GetNextCoordinate");
