#include "capture_image_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool CaptureImageNode::setRequest(Request::SharedPtr& request)
{
  // 요청을 설정하는 로직을 여기에 추가할 수 있습니다.
  return true;
}

NodeStatus CaptureImageNode::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Image captured successfully");
    return NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(logger(), "Failed to capture image: %s", response->message.c_str());
    return NodeStatus::FAILURE;
  }
}

NodeStatus CaptureImageNode::onFailure(ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Service call failed with error: %s", toStr(error));
  return NodeStatus::FAILURE;
}

// `onHalt` 메서드를 `halt` 메서드로 변경합니다.
void CaptureImageNode::halt()
{
  RCLCPP_INFO(logger(), "CaptureImageNode halted.");
  BT::RosServiceNode<std_srvs::srv::Trigger>::halt();
}

// Plugin registration
CreateRosNodePlugin(CaptureImageNode, "CaptureImageNode");
