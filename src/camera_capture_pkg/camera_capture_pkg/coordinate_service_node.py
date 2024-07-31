import rclpy
from rclpy.node import Node
from camera_capture_pkg.srv import GetNextCoordinate
from geometry_msgs.msg import Pose
import yaml
import os

class CoordinateServiceNode(Node):
    def __init__(self):
        super().__init__('coordinate_service_node')

        # 환경 변수에서 경로를 가져옵니다.
        yaml_path = os.getenv('INSPECTION_POSITIONS_PATH')
        if yaml_path is None:
            self.get_logger().error('Environment variable INSPECTION_POSITIONS_PATH is not set')
            rclpy.shutdown()
            return

        self.get_logger().info(f'YAML Path: {yaml_path}')

        # YAML 파일에서 좌표 목록을 로드합니다.
        self.coordinates_ = []
        self.load_coordinates(yaml_path)

        # 서비스 서버를 생성합니다.
        self.service_ = self.create_service(GetNextCoordinate, 'get_next_coordinate', self.handle_get_next_coordinate)
        self.current_index_ = 0

    def load_coordinates(self, path):
        try:
            with open(path, 'r') as file:
                config = yaml.safe_load(file)
                positions = config.get('positions', [])
                if not positions:
                    self.get_logger().error("No 'positions' key in YAML file")
                    return

                for position in positions:
                    pose = Pose()
                    pose.position.x = position['position']['x']
                    pose.position.y = position['position']['y']
                    pose.position.z = position['position']['z']
                    pose.orientation.x = position['orientation']['x']
                    pose.orientation.y = position['orientation']['y']
                    pose.orientation.z = position['orientation']['z']
                    pose.orientation.w = position['orientation']['w']
                    self.coordinates_.append(pose)

                self.get_logger().info(f'Loaded {len(self.coordinates_)} coordinates from YAML file')
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML Exception: {e}')
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')

    def handle_get_next_coordinate(self, request, response):
        if self.current_index_ < len(self.coordinates_):
            pose = self.coordinates_[self.current_index_]
            response.x = pose.position.x
            response.y = pose.position.y
            response.z = pose.position.z
            response.orientation_x = pose.orientation.x
            response.orientation_y = pose.orientation.y
            response.orientation_z = pose.orientation.z
            response.orientation_w = pose.orientation.w
            self.current_index_ += 1
        else:
            # 모든 좌표를 다 제공했으면, 인덱스를 초기화합니다.
            self.current_index_ = 0

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
