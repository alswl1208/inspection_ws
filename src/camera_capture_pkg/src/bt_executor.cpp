#include <rclcpp/rclcpp.hpp>
#include <py_trees_ros/trees.hpp>
#include <py_trees_ros/conversions.hpp>
#include <py_trees/behaviours.hpp>
#include <py_trees/blackboard.hpp>
#include <py_trees/composites.hpp>
#include <py_trees/decorators.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_executor");

    // Create a tree
    py_trees::BehaviorTree tree;

    // Load the tree structure from the XML file
    std::string tree_file_path = "/home/minji/inspection_ws/src/camera_capture_pkg/inspection_tree.xml"; // Change to your path
    py_trees::xml::Document document;
    document.load(tree_file_path);
    tree = py_trees_ros::from_document(document, node);

    // Run the tree
    py_trees::ros::Tree ros_tree(node, tree);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
