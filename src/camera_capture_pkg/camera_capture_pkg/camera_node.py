import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
import os
import time
import tf2_ros

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # 이미지 토픽을 구독합니다. 메시지가 수신되면 image_callback 함수를 호출합니다.
        self.subscription = self.create_subscription(
            Image,
            'LD90/camera_360/image_color',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # tf2_ros Buffer와 Listener를 초기화합니다.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # capture_image 서비스를 생성합니다. 서비스 요청이 들어오면 capture_image_callback 함수를 호출합니다.
        self.service = self.create_service(
            Trigger,
            'capture_image',
            self.capture_image_callback
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_metadata = {
            "position": {"x": 0, "y": 0, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
        }

        # 타이머를 설정하여 tf 데이터를 주기적으로 업데이트합니다.
        self.timer = self.create_timer(0.1, self.update_tf_data)

    def image_callback(self, msg):
        try:
            # 수신된 ROS 이미지 메시지를 OpenCV 이미지로 변환합니다.
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('cv_bridge exception: {}'.format(e))

    def update_tf_data(self):
        try:
            # 'odom' 프레임과 'base_link' 프레임 사이의 변환을 가져옵니다.
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.latest_metadata["position"] = {
                "x": trans.transform.translation.x,
                "y": trans.transform.translation.y,
                "z": trans.transform.translation.z
            }
            self.latest_metadata["orientation"] = {
                "x": trans.transform.rotation.x,
                "y": trans.transform.rotation.y,
                "z": trans.transform.rotation.z,
                "w": trans.transform.rotation.w
            }
            self.get_logger().info('Updated metadata: {}'.format(self.latest_metadata))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('tf2 exception: {}'.format(e))

    def capture_image_callback(self, request, response):
        if self.latest_image is not None:
            # 파일 이름에 시간을 포함시켜 고유한 파일 이름을 만듭니다.
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            image_dir = os.path.expanduser('~/inspection_ws/src/camera_capture_pkg/config/save.yaml')
            
            image_path = os.path.join(image_dir, f'image_{timestamp}.jpg')
            metadata_path = os.path.join(image_dir, f'metadata_{timestamp}.json')

            # 디렉토리가 존재하지 않으면 생성합니다.
            os.makedirs(image_dir, exist_ok=True)

            # OpenCV를 사용하여 이미지를 파일로 저장합니다.
            cv2.imwrite(image_path, self.latest_image)

            # 메타데이터를 JSON 형식으로 파일에 저장합니다.
            with open(metadata_path, 'w') as metadata_file:
                json.dump(self.latest_metadata, metadata_file)

            # 서비스 응답을 성공으로 설정합니다.
            response.success = True
            response.message = 'Image captured and metadata saved.'
        else:
            # 이미지가 없는 경우 서비스 응답을 실패로 설정합니다.
            response.success = False
            response.message = 'No image received.'

        return response

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # CameraCaptureNode 인스턴스 생성
    node = CameraCaptureNode()

    # 노드 실행
    rclpy.spin(node)

    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
