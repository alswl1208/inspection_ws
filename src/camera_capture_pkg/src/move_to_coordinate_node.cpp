#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class MoveToCoordinateNode : public rclcpp::Node
{
public:
  MoveToCoordinateNode()
  : Node("move_to_coordinate_node"), current_index_(0), is_navigating_(false)
  {
    // navigate_to_pose 액션 서버와 통신할 클라이언트 생성
    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // 패키지 경로를 가져와서 YAML 파일 경로 설정
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_capture_pkg");
    std::string inspection_position_file = package_share_directory + "/config/inspection_position.yaml";
    RCLCPP_INFO(this->get_logger(), "Loading inspection positions from file: %s", inspection_position_file.c_str());

    // YAML 파일로부터 위치 정보를 읽어옴
    if (!load_positions(inspection_position_file)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load inspection positions from file: %s", inspection_position_file.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded inspection positions.");
    
    // 첫 번째 목표를 전송
    send_goal();
  }

private:
  bool load_positions(const std::string & file_path)
  {
    try {
      RCLCPP_INFO(this->get_logger(), "Reading positions from file: %s", file_path.c_str());
      YAML::Node config = YAML::LoadFile(file_path);
      
      if (!config["positions"]) {
        RCLCPP_ERROR(this->get_logger(), "No 'positions' key found in YAML file.");
        return false;
      }

      for (const auto & node : config["positions"]) {
        if (!node["position"] || !node["orientation"]) {
          RCLCPP_ERROR(this->get_logger(), "Invalid format in YAML file. Each entry must have 'position' and 'orientation'.");
          return false;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = node["position"]["x"].as<double>();
        pose.pose.position.y = node["position"]["y"].as<double>();
        pose.pose.position.z = node["position"]["z"].as<double>();
        pose.pose.orientation.x = node["orientation"]["x"].as<double>();
        pose.pose.orientation.y = node["orientation"]["y"].as<double>();
        pose.pose.orientation.z = node["orientation"]["z"].as<double>();
        pose.pose.orientation.w = node["orientation"]["w"].as<double>();
        positions_.push_back(pose);
        RCLCPP_INFO(this->get_logger(), "Position added: x = %f, y = %f, z = %f, w = %f", 
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.w);
      }
      return true;
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
      return false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      return false;
    }
  }

  void send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    if (current_index_ >= positions_.size()) {
      RCLCPP_INFO(this->get_logger(), "No more positions to navigate to");
      return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = positions_[current_index_];
    
    // 포즈의 모든 값 출력
    RCLCPP_INFO(this->get_logger(), "Navigating to position x: %f, y: %f, z: %f, orientation w: %f", 
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, 
                goal_msg.pose.pose.position.z, goal_msg.pose.pose.orientation.w);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MoveToCoordinateNode::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveToCoordinateNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);

    is_navigating_ = true;
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    // 결과 코드에 따라 로깅
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        is_navigating_ = false;
        // 목표가 성공적으로 완료되었을 때 다음 목표로 이동
        current_index_++;
        if (current_index_ < positions_.size()) {
          // 일정 시간 간격을 두고 다음 목표 전송
          auto timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->send_goal(); }
          );
        } else {
          RCLCPP_INFO(this->get_logger(), "All positions have been navigated to.");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        is_navigating_ = false;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        is_navigating_ = false;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        is_navigating_ = false;
        break;
    }
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    if (is_navigating_) {
      RCLCPP_INFO(this->get_logger(), "Currently navigating to position: distance remaining = %f", feedback->distance_remaining);
    }
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> positions_; // 위치 목록
  size_t current_index_; // 현재 목표 인덱스
  bool is_navigating_; // 현재 목표로 이동 중인지 여부
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToCoordinateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}