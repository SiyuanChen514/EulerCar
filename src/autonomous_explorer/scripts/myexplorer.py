import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
import numpy as np
import std_msgs
import std_srvs
import std_srvs.srv
from visualization_msgs.msg import Marker, MarkerArray
from bresenham import bresenham
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action.client import ClientGoalHandle # 导入 ClientGoalHandle
from actionlib_msgs.msg import GoalStatus
import cv2
from sensor_msgs.msg import Image
import tf2_ros

from grab_box import *
from utils import transform2world
from img_detect import *

# 机械臂动作bool值将通过参数配置
LIFT = 1
LAY = 0

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('img_width', 1280),
                ('img_height', 480),
                ('grab_threshold', 0.5),
                ('orientation_threshold', 0.1),
                ('wait_for_grab_timeout', 10.0),
                ('wait_for_tf_timeout', 1.0),
                ('wait_for_servo_timeout', 1),
                ('stop_threshold', 0.5),
                ('origin_x', -3.0),
                ('origin_y', -1.0),
                ('target_dist', 20.0),
                ('dist_sigma', 10.0),
                ('path_obstacle_penalty', 0.01),
                ('info_gain_radius', 5),
                ('unknown_neighbor_threshold', 3),
                ('focal_length', 540.35),
                ('baseline', 0.057),
                ('orientation_w', 1.0)
            ]
        )

        # 获取参数
        self.img_width = self.get_parameter('img_width').get_parameter_value().integer_value
        self.img_height = self.get_parameter('img_height').get_parameter_value().integer_value
        self.grab_threshold = self.get_parameter('grab_threshold').get_parameter_value().double_value
        self.orientation_threshold = self.get_parameter('orientation_threshold').get_parameter_value().double_value
        self.wait_for_grab_timeout = self.get_parameter('wait_for_grab_timeout').get_parameter_value().double_value
        self.wait_for_tf_timeout = self.get_parameter('wait_for_tf_timeout').get_parameter_value().double_value
        self.wait_for_servo_timeout = self.get_parameter('wait_for_servo_timeout').get_parameter_value().integer_value
        self.stop_threshold = self.get_parameter('stop_threshold').get_parameter_value().double_value
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value
        self.target_dist = self.get_parameter('target_dist').get_parameter_value().double_value
        self.dist_sigma = self.get_parameter('dist_sigma').get_parameter_value().double_value
        self.path_obstacle_penalty = self.get_parameter('path_obstacle_penalty').get_parameter_value().double_value
        self.info_gain_radius = self.get_parameter('info_gain_radius').get_parameter_value().integer_value
        self.unknown_neighbor_threshold = self.get_parameter('unknown_neighbor_threshold').get_parameter_value().integer_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.orientation_w = self.get_parameter('orientation_w').get_parameter_value().double_value
        # use_sim_time=False
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        self.get_logger().info("use_sim_time set to: False")

        # 订阅地图话题
        map_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos_profile)

        # 订阅机器人位置 (Odometry)
        odom_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, odom_qos_profile)
        
        # 导航动作客户端
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 已访问前沿点集合
        self.visited_frontiers = set()

        # 地图数据
        self.map_data = None

        # 机器人位置（网格坐标）
        self.robot_position = None

        # 前沿点发布器
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # 抓取服务
        self.servo_client = self.create_client(std_srvs.srv.SetBool, 'servo_response')

        # 订阅tf2话题
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 定义坐标系
        self.camera_frame = "camera_link"
        self.world_frame = "map"

        # 添加标志位防止重复探索
        self.is_exploring = False
        # 添加标志位，指示是否正在返回原点
        self.returning_to_origin = False
        # 添加一个变量来存储当前活跃的导航目标句柄
        self.current_goal_handle: ClientGoalHandle = None

        # 定义停止探索的目标点（世界坐标）和阈值
        self.stop_threshold = 0.5
        
        # 周期性识别物块
        self.initial_box_timer = self.create_timer(
            4.0, self.process_box_callback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

        # 初始化后启动第一次探索（延迟以等待地图和机器人位置数据）
        self.initial_exploration_timer = self.create_timer(
            3.0, self.start_initial_exploration, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.get_logger().info("Waiting for map and odom data before starting exploration...")

    def start_exploration(self):
        if self.map_data is not None and self.robot_position is not None:
            self.get_logger().info("Starting first exploration after data initialization.")
            self.initial_exploration_timer.cancel()
            self.explore()
        else:
            self.get_logger().warning("Map or robot position not yet available, delaying initial exploration.")

    def odom_callback(self, msg):
        if self.map_data is None:
            return

        self.world_x = msg.pose.pose.position.x
        self.world_y = msg.pose.pose.position.y
        self.world_yaw = msg.pose.pose.orientation.z

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid_col = int((self.world_x - origin_x) / resolution)
        grid_row = int((self.world_y - origin_y) / resolution)

        rows, cols = self.map_data.info.height, self.map_data.info.width
        if 0 <= grid_row < rows and 0 <= grid_col < cols:
            self.robot_position = (grid_row, grid_col)
            '''
            if not self.returning_to_origin:
                distance_to_stop_point = np.sqrt(
                    (self.world_x - self.stop_exploration_x)**2 +
                    (self.world_y - self.stop_exploration_y)**2
                )
                if distance_to_stop_point < self.stop_threshold:
                    self.get_logger().info(
                        f"Robot is near the stop point ({self.stop_exploration_x:.2f}, {self.stop_exploration_y:.2f}) "
                        f" (distance: {distance_to_stop_point:.2f}m). Stopping exploration and returning to origin."
                    )
                    self.stop_exploration_and_return_to_origin()
            '''
        else:
            self.get_logger().warning(f"Robot world position ({self.world_x:.2f}, {self.world_y:.2f}) maps to grid ({grid_row}, {grid_col}) out of map bounds.")

    def map_callback(self, msg):
        self.map_data = msg

    def publish_frontiers(self, frontiers):
        marker_array = MarkerArray()

        if not frontiers and self.frontier_pub.get_subscription_count() > 0:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = 0
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.frontier_pub.publish(marker_array)
            return

        for i, (r, c) in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = c * self.map_data.info.resolution + self.map_data.info.origin.position.x
            marker.pose.position.y = r * self.map_data.info.resolution + self.map_data.info.origin.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            marker_array.markers.append(marker)

        self.frontier_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(frontiers)} frontier markers.")

    def navigate_to(self, x, y):
        # 验证输入参数
        if x is None or y is None or x == "" or y == "":
            self.get_logger().error("Invalid navigation goal: x or y is empty or None")
            self.is_exploring = False
            return
        # 如果正在返回原点，并且已经有活跃的导航目标，则不发送新的探索目标
        # 定义非终止状态（目标仍在处理中）
        non_terminal_states = [
            GoalStatus.PENDING,      # 0: 待处理
            GoalStatus.ACTIVE,       # 1: 正在执行
            GoalStatus.PREEMPTING,   # 6: 正在取消（执行后收到取消请求）
            GoalStatus.RECALLING     # 7: 正在取消（执行前收到取消请求）
        ]

        # 检查是否正在返回原点且有活跃目标
        if self.returning_to_origin and self.current_goal_handle is not None:
            if self.current_goal_handle.status in non_terminal_states:
                self.get_logger().info(f"Currently returning to origin with a non-terminal goal (status: {self.current_goal_handle.status}), skipping new navigation goal.")
                return
            # 如果目标已进入终止状态，重置 goal_handle
            elif self.current_goal_handle.status in [
                GoalStatus.SUCCEEDED,  # 3: 成功
                GoalStatus.ABORTED,    # 4: 失败
                GoalStatus.REJECTED,   # 5: 拒绝
                GoalStatus.PREEMPTED,  # 2: 取消后完成
                GoalStatus.RECALLED,   # 8: 执行前取消
                GoalStatus.LOST        # 9: 丢失
            ]:
                self.get_logger().info(f"Previous goal reached terminal state (status: {self.current_goal_handle.status}), resetting goal handle.")
                self.current_goal_handle = None

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.orientation.w = self.orientation_w # 机器人的面向方向可能要修改

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Sending navigation goal: x={x:.2f}, y={y:.2f}")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available, cannot send goal!")
            self.is_exploring = False
            return

        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2!")
            self.is_exploring = False
            # 如果是返回原点目标被拒绝，可能需要一些恢复机制
            if self.returning_to_origin:
                self.get_logger().error("Return to origin goal rejected. Attempting to resend or shut down.")
                # 这里可以添加重试逻辑或直接关闭
                # self.get_logger().info("Shutting down after return to origin goal rejection.")
                # self.destroy_node()
                # rclpy.shutdown()
            else:
                self.explore() # 探索目标被拒绝，尝试寻找下一个
            return
        self.get_logger().info("Goal accepted by Nav2.")
        self.current_goal_handle = goal_handle # 存储当前活跃的目标句柄
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            # 从 future 中获取结果句柄
            goal_handle = future.result()
            # 从结果句柄中获取最终状态和结果
            status = goal_handle.status
            result = goal_handle.result
            # self.get_logger().info(f"Navigation completed with status: {status}")
        except Exception as e:
            self.get_logger().error(f"Exception while getting navigation result: {e}")
            self.is_exploring = False
            self.explore() # 发生未知异常时，尝试继续探索
            return

        # 清除当前目标句柄
        self.current_goal_handle = None

        # --- 新的逻辑：根据不同的状态做出不同的反应 ---
        
        if status == GoalStatus.SUCCEEDED:
            # 导航成功
            if self.returning_to_origin:
                # 如果这是“返回原点”的导航，并且成功了
                self.get_logger().info("Robot successfully returned to origin. Exploration program terminated.")
                self.publish_frontiers([]) # 清除所有前沿点标记
                return
            else:
                # 如果这是一次常规的探索导航
                self.get_logger().info("Navigation to frontier succeeded! Looking for the next one.")
                self.is_exploring = False

                self.explore() # 成功到达一个前沿点，立即开始寻找下一个

        elif status == GoalStatus.ABORTED:
            # 导航中止（失败）
            self.get_logger().warning("Navigation SUCCEEDED. ")
            # 失败的前沿点已经被记录在 self.visited_frontiers 中，
            # 所以下次探索不会再选择它。
            # 我们直接尝试寻找一个新的、可达的前沿点。
            self.is_exploring = False
            self.explore()

        elif status == GoalStatus.PREEMPTING:
            # 导航被取消
            self.get_logger().info("Navigation was processing.")
            # 这通常是在我们调用 stop_exploration_and_return_to_origin 时发生的。
            # cancel_goal_callback 已经负责发送返回原点的目标，
            # 所以这里我们什么都不用做，只需等待返回原点的导航完成即可。
            # 将 is_exploring 设为 False，但 *不要* 调用 explore()。
            self.is_exploring = False
        
        else:
            # 其他未知状态
            self.get_logger().info(f"Navigation ended with unexpected status: {status}. Retrying exploration.")
            self.is_exploring = False
            self.explore()

    def stop_exploration_and_return_to_origin(self):
        # 检查是否已在返回原点状态
        if self.returning_to_origin:
            self.get_logger().info("Already in returning-to-origin state, skipping new request.")
            return

        # 设置返回原点状态
        self.returning_to_origin = True
        self.is_exploring = False  # 停止探索循环
        self.publish_frontiers([])  # 清除已发布的前沿点标记
        self.get_logger().info("Cleared all frontier markers and stopped exploration.")

        # 检查动作服务器是否可用
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available, cannot proceed with return to origin!")
            self.returning_to_origin = False  # 回滚状态
            self.is_exploring = True  # 恢复探索状态
            return

        # 定义非终止状态（需要取消的目标状态）
        non_terminal_states = [
            GoalStatus.PENDING,      # 0: 待处理
            GoalStatus.ACTIVE,       # 1: 正在执行
            GoalStatus.PREEMPTING,   # 6: 正在取消（执行后收到取消请求）
            GoalStatus.RECALLING,     # 7: 正在取消（执行前收到取消请求）
            GoalStatus.PREEMPTED  # 2: 正在进行
        ]

        # 检查当前目标状态
        if self.current_goal_handle is not None:
            status = self.current_goal_handle.status
            if status in non_terminal_states:
                self.get_logger().info(f"Cancelling current navigation goal (status: {status}).")
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_goal_callback)
            elif status in [
                GoalStatus.SUCCEEDED,  # 3: 成功
                GoalStatus.ABORTED,    # 4: 失败
                GoalStatus.REJECTED,   # 5: 拒绝
                GoalStatus.RECALLED,   # 8: 执行前取消
                GoalStatus.LOST        # 9: 丢失
            ]:
                self.get_logger().info(f"Current goal in terminal state (status: {status}), resetting goal handle.")
                self.current_goal_handle = None
                self._send_return_to_origin_goal()
            else:
                self.get_logger().warn(f"Unexpected goal status: {status}, resetting goal handle.")
                self.current_goal_handle = None
                self._send_return_to_origin_goal()
        else:
            self.get_logger().info("No active goal to cancel, proceeding to return to origin.")
            self._send_return_to_origin_goal()

    def cancel_goal_callback(self, future):
        """
        处理取消目标请求的结果。
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Successfully sent cancel request for the active goal.")
        else:
            self.get_logger().warning("Failed to cancel the active goal or no goal was cancelling.")
        # 无论取消成功与否，都尝试发送返回原点的目标
        self._send_return_to_origin_goal()

    def _send_return_to_origin_goal(self):
        """
        封装发送返回原点目标的逻辑。
        """
        origin_x = -3.0
        origin_y = -1.0
        self.get_logger().info(f"Navigating back to origin: x={origin_x:.2f}, y={origin_y:.2f}")
        self.navigate_to(origin_x, origin_y)

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        # Iterate through the map, excluding the outer border to simplify neighbor checks
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                # A cell is a potential frontier if it's free (0)
                if 0 <= map_array[r, c] <= 0.8:
                    # Extract the 8 neighbors around the current cell
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    # Count how many of these neighbors are unknown (-1)
                    unknown_neighbor_count = np.sum(neighbors == -1)
                    # If there are 3 or more unknown neighbors, consider it a frontier
                    if unknown_neighbor_count >= 3: # Modified condition
                        frontiers.append((r, c))
        return frontiers

    def simplify_frontiers(self, frontiers, map_array):
        """
        精简前沿点集合，移除边界点以及周围半径为2内有障碍物的点。
        """
        frontier_set = set(frontiers)
        neighbors_delta = [(-1, -1), (-1, 0), (-1, 1),
                        (0, -1),          (0, 1),
                        (1, -1),  (1, 0),  (1, 1)]
        
        # 定义半径为2的检查范围（曼哈顿距离或切比雪夫距离）
        radius_2_delta = [(dr, dc) for dr in range(-2, 3) for dc in range(-2, 3)
                        if not (dr == 0 and dc == 0)]  # 排除自身
        
        def get_neighbors(point):
            r, c = point
            return [(r + dr, c + dc) for dr, dc in neighbors_delta
                    if (r + dr, c + dc) in frontier_set]
        
        def has_obstacle_in_radius_2(point, map_array):
            r, c = point
            rows, cols = len(map_array), len(map_array[0])
            for dr, dc in radius_2_delta:
                nr, nc = r + dr, c + dc
                # 检查是否在地图范围内且是否为障碍物
                if 0 <= nr < rows and 0 <= nc < cols and map_array[nr][nc] > 0:
                    return True
            return False
        
        neighbors_dict = {point: get_neighbors(point) for point in frontiers}
        # 1 <= len(neighbors) <= 3
        while True:
            # 边界点：邻居数在1到3之间，或半径2内有障碍物
            boundary_points = [point for point, neighbors in neighbors_dict.items()
                            if (len(neighbors)>=0) or 
                                has_obstacle_in_radius_2(point, map_array)]
            if not boundary_points:
                break
            for point in boundary_points:
                frontier_set.remove(point)
                del neighbors_dict[point]
                for neighbor in neighbors_dict:
                    if point in neighbors_dict[neighbor]:
                        neighbors_dict[neighbor].remove(point)
        
        return list(frontier_set)


    def estimate_info_gain(self, map_array, frontier, radius=5):
        r, c = frontier
        rows, cols = map_array.shape
        r_min = max(0, r - radius)
        r_max = min(rows, r + radius + 1)
        c_min = max(0, c - radius)
        c_max = min(cols, c + radius + 1)
        unknown_count = np.sum(map_array[r_min:r_max, c_min:c_max] == -1)
        return unknown_count

    def choose_frontier(self, frontiers):
        if not frontiers or self.robot_position is None:
            return None

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        best_score = float('-inf')
        chosen_frontier = None
        self.target_dist = 20.0
        self.dist_sigma = 10.0
        self.path_obstacle_penalty = 0.01

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            euclidean_dist = np.sqrt((frontier[0] - self.robot_position[0])**2 +
                                     (frontier[1] - self.robot_position[1])**2)

            info_gain = self.estimate_info_gain(map_array, frontier, radius=5)

            obstacle_in_path = False
            path = list(bresenham(self.robot_position[0], self.robot_position[1],
                                frontier[0], frontier[1]))
            for (r, c) in path:
                if not (0 <= r < map_array.shape[0] and 0 <= c < map_array.shape[1]):
                    obstacle_in_path = True
                    break
                if map_array[r][c] > 0:
                    obstacle_in_path = True
                    break

            dist_weight = np.exp(-((euclidean_dist - self.target_dist) ** 2) / (2 * self.dist_sigma ** 2))
            score = info_gain * dist_weight

            if obstacle_in_path:
                score *= self.path_obstacle_penalty

            score += np.random.uniform(0.0, 0.001)

            if score > best_score:
                best_score = score
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier} with score: {best_score:.2f}, "
                                   f"distance: {np.sqrt((chosen_frontier[0] - self.robot_position[0])**2 + (chosen_frontier[1] - self.robot_position[1])**2):.2f} grid units, "
                                   f"info gain: {self.estimate_info_gain(map_array, chosen_frontier):.0f}.")
        else:
            self.get_logger().warning("No valid frontier found after scoring.")

        return chosen_frontier

    def explore(self):
        if self.is_exploring:
            self.get_logger().info("Already exploring, skipping new exploration cycle.")
            return

        if self.returning_to_origin:
            self.get_logger().info("Currently returning to origin, skipping exploration.")
            return

        if self.map_data is None or self.robot_position is None:
            self.get_logger().warning("Map data or robot position not available, cannot start exploration.")
            if not hasattr(self, 'initial_exploration_timer') or not self.initial_exploration_timer.is_ready():
                 self.initial_exploration_timer = self.create_timer(
                    3.0, self.start_initial_exploration, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
            return

        self.is_exploring = True

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        raw_frontiers = self.find_frontiers(map_array)
        if not raw_frontiers:
            self.get_logger().info("No more frontiers found. Exploration complete!")
            self.is_exploring = False
            return

        simplified_frontiers = self.simplify_frontiers(raw_frontiers, map_array)
        self.get_logger().info(f"Found {len(raw_frontiers)} raw frontiers, simplified to {len(simplified_frontiers)} actionable points.")

        self.publish_frontiers(simplified_frontiers)

        chosen_frontier = self.choose_frontier(simplified_frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No suitable frontier found to explore after filtering and scoring. Trying again...")
            self.is_exploring = False
            self.create_timer(1.0, self.explore, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        self.navigate_to(goal_x, goal_y)

    # --------------------------------物块识别----------------------------------
    
    # 主函数
    def process_box_callback(self):
        # 1. 接收相机数据,识别物块并获取坐标(失败检测)
        self.get_logger().info("receiving camera data...")  
        box_loc = process_video(self.focal_length,self.baseline,self.img_width,self.img_height,0)
        if box_loc == None:
            self.get_logger().info("Failed to detect box!")
            return
        else:
            self.get_logger().info("receiving the box corrdinates successfully!")
            # 2. 变换到世界坐标
            world_box_loc = transform2world(self,self.camera_frame,box_loc)
            self.get_logger().info("Receiving coordinates")
            # 3. 抓取模块
            if grab_lift_box(self,world_box_loc) is not None:
                # 失败是否要重新抓取
                # 4. 若抓取成功，周期性检查抓取状态

                # 5. 放置模块
                world_lay_corrds = [0,0]
                grab_lay_box(self,world_lay_corrds)
                
                # 7. 继续探索
                # 从/map，catografer获取地图信息
            
        
def main(args=None):
        rclpy.init(args=args)
        explorer_node = ExplorerNode()
        try:
            explorer_node.get_logger().info("ROS 2 Explorer Node running...")
            rclpy.spin(explorer_node)
        except KeyboardInterrupt:
            explorer_node.get_logger().info("Exploration stopped by user (Ctrl+C).")
        finally:
            if rclpy.ok() and explorer_node.frontier_pub.get_subscription_count() > 0:
                marker_array = MarkerArray()
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = explorer_node.get_clock().now().to_msg()
                marker.ns = 'frontiers'
                marker.id = 0
                marker.action = Marker.DELETEALL
                marker_array.markers.append(marker)
                explorer_node.frontier_pub.publish(marker_array)
                explorer_node.get_logger().info("Cleared all frontier markers on shutdown.")
            explorer_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':

    main()