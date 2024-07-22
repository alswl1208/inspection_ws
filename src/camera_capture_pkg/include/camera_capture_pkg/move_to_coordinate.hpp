// MoveToCoordinate.hpp
#ifndef CAMERA_CAPTURE_PKG_MOVE_TO_COORDINATE_HPP
#define CAMERA_CAPTURE_PKG_MOVE_TO_COORDINATE_HPP

#include "behaviortree_cpp_v3/action_node.h"

class MoveToCoordinate : public BT::SyncActionNode {
public:
    MoveToCoordinate(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    bool move_to_coordinate_service(const std::string& target_coordinates);
};

#endif // CAMERA_CAPTURE_PKG_MOVE_TO_COORDINATE_HPP
