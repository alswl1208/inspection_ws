#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

class CameraCaptureNode : public rclcpp::Node
{
public:
  CameraCaptureNode() : Node("camera_capture_node")
  {
    // 이미지 토픽을 구독합니다. 메시지가 수신되면 image_callback 함수를 호출합니다.
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "LD90/camera_360/image_color", 10,
      std::bind(&CameraCaptureNode::image_callback, this, std::placeholders::_1));

    // capture_image 서비스를 생성합니다. 서비스 요청이 들어오면 capture_image_callback 함수를 호출합니다.
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_image", std::bind(&CameraCaptureNode::capture_image_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  // 이미지 콜백 함수
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // 수신된 ROS 이미지 메시지를 OpenCV 이미지로 변환합니다.
      latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
      // 임의의 메타데이터를 생성합니다.
      latest_metadata_ = {
        {"position", {{"x", 0}, {"y", 0}, {"z", 0}}},
        {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 1}}}
      };
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  // 서비스 콜백 함수
  void capture_image_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!latest_image_.empty())
    {
      // 이미지를 저장할 경로와 메타데이터를 저장할 경로를 지정합니다.
      std::string image_path = "/home//minji/inspection_ws/image.jpg";
      std::string metadata_path = "/home//minji/inspection_ws/metadata.json";

      // OpenCV를 사용하여 이미지를 파일로 저장합니다.
      cv::imwrite(image_path, latest_image_);

      // 메타데이터를 JSON 형식으로 파일에 저장합니다.
      std::ofstream metadata_file(metadata_path);
      metadata_file << latest_metadata_.dump();
      metadata_file.close();

      // 서비스 응답을 성공으로 설정합니다.
      response->success = true;
      response->message = "Image captured and metadata saved.";
    }
    else
    {
      // 이미지가 없는 경우 서비스 응답을 실패로 설정합니다.
      response->success = false;
      response->message = "No image received.";
    }
  }

  // ROS2 이미지 토픽 구독자
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  // capture_image 서비스
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  // 최신 이미지
  cv::Mat latest_image_;
  // 최신 메타데이터
  nlohmann::json latest_metadata_;
};

int main(int argc, char* argv[])
{
  // ROS2 초기화
  rclcpp::init(argc, argv);
  // CameraCaptureNode 인스턴스 생성
  auto node = std::make_shared<CameraCaptureNode>();
  // 노드 실행
  rclcpp::spin(node);
  // ROS2 종료
  rclcpp::shutdown();
  return 0;
}
