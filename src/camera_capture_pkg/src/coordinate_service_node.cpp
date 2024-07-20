#include "rclcpp/rclcpp.hpp"
#include "camera_capture_pkg/srv/get_next_coordinate.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

class CoordinateServiceNode : public rclcpp::Node
{
public:
  CoordinateServiceNode() : Node("coordinate_service_node")
  {
    // 환경 변수에서 경로를 가져옵니다.
    const char* yaml_path = std::getenv("INSPECTION_POSITIONS_PATH");
    if (yaml_path == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Environment variable INSPECTION_POSITIONS_PATH is not set");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "YAML Path: %s", yaml_path); // 추가된 부분
    // YAML 파일에서 좌표 목록을 로드합니다.
    load_coordinates(yaml_path);

    // 서비스 서버를 생성합니다.
    service_ = this->create_service<camera_capture_pkg::srv::GetNextCoordinate>(
      "get_next_coordinate", std::bind(&CoordinateServiceNode::handle_get_next_coordinate, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void load_coordinates(const std::string &path)
{
  try {
    YAML::Node config = YAML::LoadFile(path);
    auto positions = config["positions"]; // positions 키를 가져옵니다.
    if (!positions) {
      RCLCPP_ERROR(this->get_logger(), "No 'positions' key in YAML file");
      return;
    }
    for (const auto& position : positions)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = position["position"]["x"].as<double>();
      pose.position.y = position["position"]["y"].as<double>();
      pose.position.z = position["position"]["z"].as<double>();
      pose.orientation.x = position["orientation"]["x"].as<double>();
      pose.orientation.y = position["orientation"]["y"].as<double>();
      pose.orientation.z = position["orientation"]["z"].as<double>();
      pose.orientation.w = position["orientation"]["w"].as<double>();
      coordinates_.emplace_back(pose);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu coordinates from YAML file", coordinates_.size());
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}

  void handle_get_next_coordinate(
    const std::shared_ptr<camera_capture_pkg::srv::GetNextCoordinate::Request> request,
    std::shared_ptr<camera_capture_pkg::srv::GetNextCoordinate::Response> response)
  {
    if (current_index_ < coordinates_.size())
    {
      const auto& pose = coordinates_[current_index_];
      response->x = pose.position.x;
      response->y = pose.position.y;
      response->z = pose.position.z;
      response->w = pose.orientation.w;
      current_index_++;
    }
    else
    {
      // 모든 좌표를 다 제공했으면, 인덱스를 초기화합니다.
      current_index_ = 0;
    }
  }

  rclcpp::Service<camera_capture_pkg::srv::GetNextCoordinate>::SharedPtr service_;
  std::vector<geometry_msgs::msg::Pose> coordinates_;
  size_t current_index_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoordinateServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
