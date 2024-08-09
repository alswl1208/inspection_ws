import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from std_srvs.srv import Trigger
from camera_capture_pkg.srv import GetNextCoordinate
from nav2_msgs.action import NavigateToPose
from camera_capture_pkg.action import ExecuteTree
from geometry_msgs.msg import PoseStamped
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees_ros.trees import BehaviourTree
from rclpy.action import GoalResponse, CancelResponse, GoalEvent

class GetNextCoordinateNode(Behaviour):
    def __init__(self, node):
        super(GetNextCoordinateNode, self).__init__("GetNextCoordinate")
        self.node = node
        self.cli = self.node.create_client(GetNextCoordinate, '/get_next_coordinate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service /get_next_coordinate not available, waiting again...')

    def update(self):
        self.node.get_logger().info('Updating GetNextCoordinateNode...')
        request = GetNextCoordinate.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response:
            self.node.get_logger().info(f'Got next coordinate: ({response.x}, {response.y}, {response.z}), orientation: ({response.orientation_x}, {response.orientation_y}, {response.orientation_z}, {response.orientation_w})')
            self.node.coordinate = {
                'x': response.x, 'y': response.y, 'z': response.z,
                'orientation_x': response.orientation_x, 'orientation_y': response.orientation_y,
                'orientation_z': response.orientation_z, 'orientation_w': response.orientation_w
            }
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error('Failed to get next coordinate')
            return py_trees.common.Status.FAILURE

class MoveToCoordinateNode(Behaviour):
    def __init__(self, node):
        super(MoveToCoordinateNode, self).__init__("MoveToCoordinate")
        self.node = node
        self.cli = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server /navigate_to_pose not available, waiting again...')

    def update(self):
        self.node.get_logger().info('Updating MoveToCoordinateNode...')
        
        # Get the latest coordinate from stored location
        if not hasattr(self.node, 'coordinate'):
            self.node.get_logger().error('No coordinate available')
            return py_trees.common.Status.FAILURE
        
        coordinate = self.node.coordinate

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coordinate['x']
        goal_msg.pose.pose.position.y = coordinate['y']
        goal_msg.pose.pose.position.z = coordinate['z']
        goal_msg.pose.pose.orientation.x = coordinate['orientation_x']
        goal_msg.pose.pose.orientation.y = coordinate['orientation_y']
        goal_msg.pose.pose.orientation.z = coordinate['orientation_z']
        goal_msg.pose.pose.orientation.w = coordinate['orientation_w']

        self.node.get_logger().info(f'Sending goal request to navigate to position: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}, {goal_msg.pose.pose.position.z}')

        self._send_goal_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, self._send_goal_future)
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error('MoveToCoordinate goal rejected')
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info('MoveToCoordinate goal accepted, waiting for result...')

        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self._get_result_future)
        result = self._get_result_future.result()

        if result.status != GoalEvent.SUCCEEDED:
            self.node.get_logger().error(f'Failed to navigate to coordinate, status: {result.status}')
            if result.result:
                self.node.get_logger().error(f'Navigation result: {result.result}')
            if result.status == GoalEvent.ABORT:
                self.node.get_logger().error('Goal was aborted. Possible causes: unreachable location, incorrect goal parameters, or navigation system issues.')
            elif result.status == GoalEvent.CANCEL_GOAL:
                self.node.get_logger().error('Goal was canceled. Possible causes: user intervention or navigation system issues.')
            else:
                self.node.get_logger().error('Unknown result code')
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info('Navigation succeeded')
        return py_trees.common.Status.SUCCESS

class CaptureImageNode(Behaviour):
    def __init__(self, node):
        super(CaptureImageNode, self).__init__("CaptureImage")
        self.node = node
        self.cli = self.node.create_client(Trigger, '/capture_image')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service /capture_image not available, waiting again...')

    def update(self):
        self.node.get_logger().info('Updating CaptureImageNode...')
        request = Trigger.Request()
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response.success:
            self.node.get_logger().info('Image captured successfully')
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error('Failed to capture image')
            return py_trees.common.Status.FAILURE

class MoveToHomeNode(Behaviour):
    def __init__(self, node):
        super(MoveToHomeNode, self).__init__("MoveToHome")
        self.node = node
        self.cli = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        while not self.cli.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Action server /navigate_to_pose not available, waiting again...')

    def update(self):
        self.node.get_logger().info('Updating MoveToHomeNode...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.0  # replace with home coordinate
        goal_msg.pose.pose.position.y = 0.0  # replace with home coordinate
        goal_msg.pose.pose.position.z = 0.0  # replace with home coordinate
        goal_msg.pose.pose.orientation.x = 0.0  # replace with home coordinate
        goal_msg.pose.pose.orientation.y = 0.0  # replace with home coordinate
        goal_msg.pose.pose.orientation.z = 0.0  # replace with home coordinate
        goal_msg.pose.pose.orientation.w = 1.0  # replace with home coordinate

        self._send_goal_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, self._send_goal_future)
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error('MoveToHome goal rejected')
            return py_trees.common.Status.FAILURE

        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self._get_result_future)
        result = self._get_result_future.result()

        if result.status != GoalEvent.SUCCEEDED:
            self.node.get_logger().error('Failed to move to home')
            return py_trees.common.Status.FAILURE

        self.node.get_logger().info('Move to home succeeded')
        return py_trees.common.Status.SUCCESS

class BehaviorTreeActionServer(Node):
    def __init__(self):
        super().__init__('behavior_tree_action_server')
        self._action_server = ActionServer(
            self,
            ExecuteTree,
            'execute_tree',
            self.execute_callback
        )
        self.tree = None
        self.get_logger().info('BehaviorTreeActionServer started')

    def create_tree(self):
        root = Sequence("Main Sequence", memory=True)

        get_next_coordinate = GetNextCoordinateNode(self)
        move_to_coordinate = MoveToCoordinateNode(self)
        capture_image = CaptureImageNode(self)
        move_to_home = MoveToHomeNode(self)

        root.add_children([get_next_coordinate, move_to_coordinate, capture_image, move_to_home])

        tree = BehaviourTree(root)
        tree.setup(timeout=15)  # 트리 설정
        tree.node = self  # 노드 객체를 전달
        self.get_logger().info('Tree created and set up')
        return tree

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing tree: {goal_handle.request.target_tree}')
        self.tree = self.create_tree()

        # 주기적으로 트리를 틱(Tick)합니다.
        while rclpy.ok():
            self.tree.tick()  # 트리 틱(Tick)
            self.get_logger().info(f'Tree status: {self.tree.root.status}')
            for node in self.tree.root.iterate():
                self.get_logger().info(f'Node {node.name} status: {node.status}')
            if self.tree.root.status != py_trees.common.Status.RUNNING:
                break
            await rclpy.sleep(0.5)  # 500ms 대기 후 다시 틱

        result = ExecuteTree.Result()
        if self.tree.root.status == py_trees.common.Status.SUCCESS:
            result.error_message = ''
        else:
            result.error_message = 'Failed to execute tree'

        goal_handle.succeed()
        goal_handle.result = result
        return result

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

