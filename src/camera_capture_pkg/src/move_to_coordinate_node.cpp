#include "camera_capture_pkg/move_to_coordinate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_client/Client.hpp"

// MoveToCoordinate 노드 구현
BT::NodeStatus MoveToCoordinate(BT::TreeNode& self) {
    auto target_coordinates = self.getInput<std::string>("target_coordinates");
    if (!target_coordinates) {
        throw BT::RuntimeError("missing required input [target_coordinates]");
    }

    // 좌표로 이동하는 Nav2 서비스 호출
    if (move_to_coordinate_service(*target_coordinates)) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

bool move_to_coordinate_service(const std::string& target_coordinates) {
    // Nav2 서비스 호출 구현
    return true; // 서비스 호출 성공 여부에 따라 true 또는 false 반환
}
