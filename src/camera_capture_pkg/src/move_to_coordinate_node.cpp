#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

// 지정된 좌표로 이동하는 로봇 노드를 정의
class MoveToCoordinateNode : public rclcpp::Node
{
public:
  // 액션 클라이언트를 생성
  MoveToCoordinateNode()
  : Node("move_to_coordinate_node"), current_index_(0)
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

    // 로드된 위치 목록을 로깅
    for (size_t i = 0; i < positions_.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Loaded position %zu: x = %f, y = %f, z = %f, w = %f", 
                  i, positions_[i].pose.position.x, positions_[i].pose.position.y, 
                  positions_[i].pose.position.z, positions_[i].pose.orientation.w);
    }

    // 1초마다 send_goal 메소드를 호출하는 타이머 생성
    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MoveToCoordinateNode::send_goal, this));
  }

private:
  // YAML 파일로부터 위치 정보를 읽어오는 메소드
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

  // 목표 좌표로 이동하는 goal을 전송하는 메소드
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

    // 목표 메시지 생성 및 YAML 파일에서 값 가져오기
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = positions_[current_index_];
    RCLCPP_INFO(this->get_logger(), "Navigating to position x: %f, y: %f", 
                goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

    // 목표 전송 옵션 설정 및 결과 콜백 설정
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MoveToCoordinateNode::result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MoveToCoordinateNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    current_index_++;
  }

  // 목표 결과에 대한 콜백 함수
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    // 결과 코드에 따라 로깅
    switch(result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  // 목표 진행 상황에 대한 피드백 콜백 함수
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance remaining = %f", feedback->distance_remaining);
  }

  // 액션 클라이언트와 타이머를 위한 멤버 변수
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> positions_; // 위치 목록
  size_t current_index_; // 현재 목표 인덱스
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToCoordinateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
