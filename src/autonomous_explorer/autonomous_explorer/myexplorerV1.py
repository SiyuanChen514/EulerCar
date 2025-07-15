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
from transformations import euler_from_quaternion
from transformations import quaternion_from_euler
import tf2_ros

from grab_box import *
from utils import transform2world
from autonomous_explorer.autonomous_explorer.img_detectV1 import *

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
        self.processed_map_array = None

        # 机器人位置与姿态（世界坐标）
        self.world_x = None
        self.world_y = None
        self.robot_quaternion = None
        self.world_yaw = None

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
        self.camera_frame = "double_camera_link"
        self.world_frame = "map"

        self.print_state_count = 0
        # 添加标志位防止重复探索
        self.is_exploring = False

        self.green_ok = False
        self.red_ok = False
        self.blue_ok = False

        # 添加标志位，指示是否正在返回原点
        self.returning_to_origin = False
        # 添加一个变量来存储当前活跃的导航目标句柄
        self.current_goal_handle: ClientGoalHandle = None

        # 定义停止探索的目标点（世界坐标）和阈值
        self.stop_threshold = 0.5
        
        # 存放当前导航目标
        self.nav_goal = []

        self.green_loc = []
        self.red_loc = []
        self.blue_loc = []

        # 是否检测到物块开始抓取
        self.beginGrab = "no"

        self.ready_to_put = False
        self.put_ok = False

        self.grabbed = False
        # 状态机定时器
        self.check_StateMachine_timer = self.create_timer(0.01, self.check_state_machine, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.car_state = "init"
        
        # 周期性识别物块
        self.initial_box_timer = self.create_timer(
            4.0, self.detect_box_callback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

    def preprocess_map_for_frontiers(self, map_data):
        """
        预处理地图数据，仅用于前沿点检测。
        将值 > 50 设为 1，<= 50 设为 0，保留 -1。
        参数:
            map_data: nav_msgs/OccupancyGrid 类型的地图数据
        返回:
            processed_array: 预处理后的 numpy 数组
        """
        # 将地图数据转换为 numpy 数组
        map_array = np.array(map_data.data).reshape(
            (map_data.info.height, map_data.info.width))

        # 创建输出数组，保留原始形状
        processed_array = np.copy(map_array)

        # 预处理逻辑：> 50 设为 1，<= 50 设为 0，保留 -1
        processed_array[(map_array > 50) & (map_array != -1)] = 1
        processed_array[(map_array <= 50) & (map_array != -1)] = 0

        # 检查是否有无效值
        if np.any(np.isnan(processed_array)) or np.any(np.isinf(processed_array)):
            self.get_logger().warning("Processed map contains NaN or Inf values, returning original map")
            return map_array

        #self.get_logger().info(f"Preprocessed map for frontiers: {np.sum(processed_array == -1)} unknown, "
                              #f"{np.sum(processed_array == 0)} free, {np.sum(processed_array == 1)} occupied")
        return processed_array

    def check_state_machine(self):

        # 定时打印当前状态
        self.print_state_count += 1
        if self.print_state_count == 100:
            print(f"Current state: {self.car_state}")
            self.print_state_count = 0

        # 初始时等待地图和机器人位置数据
        if self.car_state == "init":
            if self.map_data is not None and self.robot_position is not None:
                # 启动探索
                self.change_state("explore")
                return 
            else:
                print("Waiting for map and odom data")
                return
        if self.car_state == "explore":
            # 如果存在探索点则进行导航
            if self.nav_goal:
                self.change_state("moving")
                return
            else:
                self.explore()
                return  
        if self.car_state == "moving":
            # 检测获取到世界坐标并进行抓取
            if self.beginGrab:
                    self.change_state("grab")
                    self.beginGrab = False
                    return
            # 检测是否到达目标点
            # 鲁棒性
            try:
                if (self.robot_position[0] - self.nav_goal[0])**2 + (self.robot_position[1] - self.nav_goal[1])**2 < 0.005:
                    self.change_state("explore")
                    return
                else:
                    self.navigate_to(self.nav_goal,self.robot_quaternion)
                    return
            except Exception as e:
                self.get_logger().info("Error in moving to the goal:" + str(e))
        
        if self.car_state == "put_down":
            if self.put_ok:
                self.ready_to_put = False
                self.put_ok = False
                if self.red_loc != []:
                    self.nav_goal = self.red_loc
                    self.nav_attutide = self.red_attitude
                    self.change_state("moving")
                

    
        if self.car_state == "grab_green":
            if self.grabbed:
                self.green_ok = True
                self._return =  "green" 
                self.change_state("return")
            else:
                return
        if self.car_state == "grab_red":
            if self.grabbed:
                self.red_ok = True
                self._return =  "red" 
                self.change_state("return")
            else:
                return
        if self.car_state == "grab_blue":
            if self.grabbed:
                self.blue_ok = True
                self._return =  "blue" 
                self.change_state("return")
            else:
                return         
        if self.car_state == "return":
            self.grabbed = False
            self.beginGrab = False
            # self.change_state("moving")
            # home_p = [0,0,0]
            # home_q = [1,0,0,0]
            if self.ready_to_put:
                self.change_state("put_down")
            self.nav_goal = [0,0,0]
            self.navigate_to(self.nav_goal,self.robot_quaternion)

        


    # 状态机切换
    def change_state(self, state):
        self.car_state = state


    def odom_callback(self, msg):
        if self.map_data is None:
            return

        self.world_x = msg.pose.pose.position.x
        self.world_y = msg.pose.pose.position.y

        self.robot_quaternion = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        self.world_yaw = euler_from_quaternion(self.robot_quaternion)[2]

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid_col = int((self.world_x - origin_x) / resolution)
        grid_row = int((self.world_y - origin_y) / resolution)

        rows, cols = self.map_data.info.height, self.map_data.info.width
        if -rows <= grid_row < rows and -cols <= grid_col < cols:
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
        """
        接收并存储地图数据，调用预处理函数，初始化机器人位置。
        """
        self.map_data = msg
        # 为前沿点检测预处理地图
        self.processed_map_array = self.preprocess_map_for_frontiers(msg)
        if self.robot_position is None:
            # 初始化机器人位置为地图中心（网格坐标）
            rows, cols = msg.info.height, msg.info.width
            self.robot_position = (rows // 2, cols // 2)
            self.get_logger().info(f"Robot position initialized at grid: {self.robot_position}")
        self.get_logger().info("Map received and preprocessed for frontier detection")

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

    def navigate_to(self,p,q):
        """
        p为位置,q为四元数,发布在世界坐标系下的机器人预期的位置和姿态,注意p和q是地图坐标系下的坐标

        """
        # 验证输入参数
        if not p or not q:
            self.get_logger().error("Invalid navigation goal: p or q is empty or None")
            return          
        
        # non_terminal_states = [
        #     GoalStatus.PENDING,      # 0: 待处理
        #     GoalStatus.ACTIVE,       # 1: 正在执行
        #     GoalStatus.PREEMPTING,   # 6: 正在取消（执行后收到取消请求）
        #     GoalStatus.RECALLING     # 7: 正在取消（执行前收到取消请求）
        # ]

        # # 检查是否正在返回原点且有活跃目标
        # if self.returning_to_origin and self.current_goal_handle is not None:
        #     if self.current_goal_handle.status in non_terminal_states:
        #         self.get_logger().info(f"Currently returning to origin with a non-terminal goal (status: {self.current_goal_handle.status}), skipping new navigation goal.")
        #         return
        #     # 如果目标已进入终止状态，重置 goal_handle
        #     elif self.current_goal_handle.status in [
        #         GoalStatus.SUCCEEDED,  # 3: 成功
        #         GoalStatus.ABORTED,    # 4: 失败
        #         GoalStatus.REJECTED,   # 5: 拒绝
        #         GoalStatus.PREEMPTED,  # 2: 取消后完成
        #         GoalStatus.RECALLED,   # 8: 执行前取消
        #         GoalStatus.LOST        # 9: 丢失
        #     ]:
        #         self.get_logger().info(f"Previous goal reached terminal state (status: {self.current_goal_handle.status}), resetting goal handle.")
        #         self.current_goal_handle = None

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(p[0])
        goal_msg.pose.position.y = float(q[1])

        goal_msg.pose.orientation.w = float(q[0])
        goal_msg.pose.orientation.x = float(q[1])
        goal_msg.pose.orientation.y = float(q[2])
        goal_msg.pose.orientation.z = float(q[3])
        

        explore_goal = NavigateToPose.Goal()
        explore_goal.pose = goal_msg

        self.get_logger().info(f"Sending navigation goal: x={p[0]:.2f}, y={p[1]:.2f}")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available, cannot send goal!")
            self.is_exploring = False
            return

        send_goal_future = self.nav_to_pose_client.send_goal_async(explore_goal)
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
        """
        检测地图中的前沿点。
        前沿点定义为值为 -1 的未知点，其邻域内包含已知点或障碍物（0 <= x < 1）。
        """
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # 检查未知点
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    # 检查邻域内是否有已知点或障碍物（0 <= x < 1）
                    if any(n == -1 for n in neighbors):
                        frontiers.append((r, c))
        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def simplify_frontiers(self, frontiers, map_array):
        """
        精简前沿点集合，移除边界点以及周围半径为2内有障碍物的点。
        使用预处理后的地图。
        """
        frontier_set = set(frontiers)
        neighbors_delta = [(-1, -1), (-1, 0), (-1, 1),
                          (0, -1),          (0, 1),
                          (1, -1),  (1, 0),  (1, 1)]
        radius_2_delta = [(dr, dc) for dr in range(-2, 3) for dc in range(-2, 3)
                          if not (dr == 0 and dc == 0)]
        
        def get_neighbors(point):
            r, c = point
            return [(r + dr, c + dc) for dr, dc in neighbors_delta
                    if (r + dr, c + dc) in frontier_set]
        
        def has_obstacle_in_radius_2(point, map_array):
            r, c = point
            rows, cols = len(map_array), len(map_array[0])
            for dr, dc in radius_2_delta:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and map_array[nr][nc] == 1:  # 使用预处理后的障碍物值
                    return True
            return False
        
        neighbors_dict = {point: get_neighbors(point) for point in frontiers}
        while True:
            boundary_points = [point for point, neighbors in neighbors_dict.items()
                               if (1 <= len(neighbors) <= 3) or 
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
        """
        优化选择前沿点：基于信息增益和欧几里得距离。
        使用预处理后的地图。
        """
        if not frontiers:
            return None

        map_array = self.processed_map_array  # 使用预处理后的地图
        best_score = float('-inf')
        chosen_frontier = None
        target_dist = 20.0
        dist_sigma = 5.0
        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            euclidean_dist = np.sqrt((frontier[0] - self.robot_position[0])**2 + 
                                     (frontier[1] - self.robot_position[1])**2)
            info_gain = self.estimate_info_gain(map_array, frontier, radius=5)
            path = list(bresenham(self.robot_position[0], self.robot_position[1], 
                                  frontier[0], frontier[1]))
            obstacle_in_path = False
            for (nr, nc) in path:
                if map_array[nr][nc] == 1:  # 使用预处理后的障碍物值
                    obstacle_in_path = True
                    break
            dist_weight = np.exp(-((euclidean_dist - target_dist) ** 2) / (2 * dist_sigma ** 2))
            score = info_gain * dist_weight
            if obstacle_in_path:
                score *= 0.1
            score += np.random.uniform(0.0, 0.1)
            if score > best_score:
                best_score = score
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier} with score: {best_score}")
        else:
            self.get_logger().warning("No valid frontier found")

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
            return

        self.is_exploring = True
        frontiers = self.find_frontiers(self.processed_map_array)
        if not frontiers:
            self.get_logger().info("No more frontiers found. Exploration complete!")
            self.is_exploring = False
            return

        simplified_frontiers = self.simplify_frontiers(frontiers, self.processed_map_array)
        self.publish_frontiers(simplified_frontiers)
        self.get_logger().info(f"Found {len(frontiers)} frontiers, simplified to {len(simplified_frontiers)} actionable points.")

        chosen_frontier = self.choose_frontier(simplified_frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No suitable frontier found to explore after filtering and scoring. Trying again...")
            self.is_exploring = False
            self.create_timer(1.0, self.explore, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
            return

        self.nav_goal[0] = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        self.nav_goal[1] = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        # self.navigate_to(goal_x, goal_y)

    # --------------------------------物块识别----------------------------------
    
    # 主函数 
    def detect_box_callback(self):


        """
        定时器回调函数，用于检测物块并赋值位置
        赋值标志beginGrab
        self.world_box_loc: 世界坐标系下的物块坐标
        参数:
            None
        返回值:
            无
        """

        # 先进行检测查看是否检测到物体，并记录检测到物体的位置减少探索时间,如果已经记录物体位置或者搬运过这个物体就跳过。
        if self.green_loc == [] and not self.green_ok:
            _,_,green_detected_ = \
            process_video("green",self.focal_length,self.baseline,self.img_width,self.img_height,0)
            if green_detected_ == True:
                self.BeginGrab = "green"

        if self.red_loc ==[] and not self.red_ok:
            _,_,red_detected_ = \
            process_video("red",self.focal_length,self.baseline,self.img_width,self.img_height,0)
            if red_detected_ == True:
                self.red_loc = [self.world_x,self.world_y]
                self.red_attitude = self.robot_quaternion

            # 如果已经搬运完绿色物体并且检测到红色物体，则开始搬运红色物体
            if self.green_ok and not self.red_ok:
                self.BeginGrab = "red"
        
        if self.blue_loc == [] and not self.blue_ok:
            _,_,blue_detected_ = \
            process_video("blue",self.focal_length,self.baseline,self.img_width,self.img_height,0)
            if blue_detected_ == True:            
                self.blue_loc = [self.world_x,self.world_y]
                self.blue_attitude = self.robot_quaternion

            # 如果已经搬运完绿色和红色物体并且检测到蓝色物体，则开始搬运蓝色物体
            if self.green_ok and self.red_ok and not self.blue_ok:
                self.BeginGrab = "blue"

        if self.car_state == "return":
            return 
        if self.car_state == "grab_green":
            color = "green"
        elif self.car_state == "grab_red":
            color = "red"
        elif self.car_state == "grab_blue":
            color = "blue"

        # 1. 接收相机数据,识别物块并获取坐标(失败检测)
        self.get_logger().info("receiving camera data...")  
        box_loc,grabbed_,_ = process_video(color,self.focal_length,self.baseline,self.img_width,self.img_height,0)
        if grabbed_:
            self.grabbed = True
            """
            这里加上抓取的代码，并且需要加上一定的延时函数保证抓取成功


            """
            return 
        self.grab_adjust(box_loc)      
        
          
    def grab_adjust(self,box_loc):

        if box_loc == []:
            self.get_logger().info("Detect no box!")
            return
        else:
            self.beginGrab = True
            self.get_logger().info("receiving the box corrdinates successfully!")
            # 检查是否进入抓取状态，避免和moving状态控制冲突
            if self.car_state == 'grab':            
            # 进行抓取操作
                if abs(box_loc[0]) <= 0.02 and abs(box_loc) < 0.14:
                    print("-----------------------Ready to Grab---------------------")
                    # p为位置，q为姿态
                    # robot_location的x轴代表box_location的y轴
                    x_ = np.cos(self.world_yaw)* 0.15 + self.robot_position[0]
                    y_ = np.sin(self.world_yaw)* 0.15 + self.robot_position[1]
                    p = [x_,y_]

                    q = self.robot_quaternion
                    # self.box_location = [-1,-1,-1]
                    # # self.beginGrab = False
                    # # self.grabbing = Truep
                    self.navigate_to(p,q)

                elif abs(box_loc[0]) <= 0.02 and abs(box_loc[1]) >= 0.14:
                    print("-----------------------Too Far---------------------")
                    # p为位置，q为姿态
                    x_ = np.cos(self.world_yaw)*(box_loc[1] - 0.12) + self.robot_position[0]
                    y_ = np.sin(self.world_yaw)*(box_loc[1] - 0.12) + self.robot_position[1]
                    p = [x_,y_]

                    q = self.robot_quaternion
                    # self.box_location = [-1,-1,-1]
                    self.navigate_to(p,q)
            
                elif abs(box_loc[0]) > 0.02:
                    print("-----------------------Adjust Attitude---------------------")
                    # 计算物体相对于机器人的yaw
                    if box_loc[1] == 0:
                        if box_loc[0] > 0:
                            yaw_delta = np.pi/2
                        else:
                            yaw_delta = -np.pi/2
                    else:
                        yaw_delta = np.arctan(box_loc[0]/box_loc[1])

                    # yaw_robot = euler_from_quaternion(self.robot_attitude)[2]


                    yaw_goal = self.world_yaw - yaw_delta

                    print(self.yaw_robot)
                    print(f"you should turn right {yaw_delta*180/np.pi} degree!!!!!!!!!!!!!!")
                    # 转换为四元数
                    p = [self.robot_position[1],self.robot_position[0],0]
                    q = quaternion_from_euler(0,0,yaw_goal)
                    # self.box_location = [-1,-1,-1]

                    self.navigate_to(p,q)        
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