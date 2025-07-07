import rclpy
import numpy as np
# from .grab_box import *
from .img_detectV1 import *
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from transformations import quaternion_from_euler
from transformations import euler_from_quaternion


class GrabNode(Node):
    
    def __init__(self):

        super().__init__('grab_node')
        # 相机内参
        self.focal_length = 500
        self.baseline = 0.057
        self.img_width = 1280
        self.img_height = 480

        # 是否接受定位信息
        self.init = False

        self.beginGrab = False

        self.grabbing = False

        self.robot_location = [0,0]
        self.robot_attitude = [0,0,0,1]

        self.box_location = [-1,-1,-1]

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        odom_qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # self.publisher_ = self.create_publisher(   , 'grab_topic', odom_qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, odom_qos_profile)
        
        self.DetectBox_timer = self.create_timer(
            0.5, self.detectCallback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        
        # self.grabTimer = self.create_timer(
        #     0.1, self.grabCallback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

    def odom_callback(self, msg):
        self.init  = True
        self.robot_location[0] = msg.pose.pose.position.x
        self.robot_location[1] = msg.pose.pose.position.y

        self.robot_attitude[0] = msg.pose.pose.orientation.x
        self.robot_attitude[1] = msg.pose.pose.orientation.y
        self.robot_attitude[2] = msg.pose.pose.orientation.z
        self.robot_attitude[3] = msg.pose.pose.orientation.w
        # self.world_yaw = msg.pose.pose.orientation.z

    def detectCallback(self):
        self.get_logger().info("receiving camera data...")  
        self.box_location = process_video(self.focal_length,self.baseline,self.img_width,self.img_height,2)
        if self.box_location == [-1,-1,-1] or self.box_location == []:
            self.get_logger().info("Failed to detect box!")
            return
        else:

            if not self.init:
                return
            

            # 进行抓取操作
            if abs(self.box_location[0]) <= 0.02 and abs(self.box_location[1]) < 0.14:
                print("-----------------------Ready to Grab---------------------")
                # p为位置，q为姿态
                # robot_location的x轴代表box_location的y轴
                p = [self.robot_location[1],0.15+self.robot_location[0],0]
                q = self.robot_attitude
                # self.box_location = [-1,-1,-1]
                # # self.beginGrab = False
                # # self.grabbing = Truep
                self.Navigate2(p,q)

            elif abs(self.box_location[0]) <= 0.02 and abs(self.box_location[1]) >= 0.14:
                print("-----------------------Too Far---------------------")
                # p为位置，q为姿态
                p = [self.robot_location[1],self.box_location[1]-0.12+self.robot_location[0],0]
                q = self.robot_attitude
                # self.box_location = [-1,-1,-1]
                self.Navigate2(p,q)
         
            elif abs(self.box_location[0]) > 0.02:
                print("-----------------------Adjust Attitude---------------------")
                # 计算物体相对于机器人的yaw
                if self.box_location[1] == 0:
                    if self.box_location[0] > 0:
                        yaw_delta = np.pi/2
                    else:
                        yaw_delta = -np.pi/2
                else:
                    yaw_delta = np.arctan(self.box_location[0]/self.box_location[1])

                yaw_robot = euler_from_quaternion(self.robot_attitude)[2]


                yaw_goal = yaw_robot - yaw_delta

                print(yaw_robot)
                print(f"you should turn right {yaw_goal*180/np.pi} degree!!!!!!!!!!!!!!")
                # 转换为四元数
                p = [self.robot_location[1],self.robot_location[0],0]
                q = quaternion_from_euler(0,0,yaw_goal)
                # self.box_location = [-1,-1,-1]

                self.Navigate2(p,q)
            self.box_location[1]=self.box_location[2]
            self.beginGrab = True
            self.get_logger().info("receiving the box corrdinates successfully!")

        

    def Navigate2(self,p,q):
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        # 将相机坐标系下的坐标转换为map坐标系下的坐标
        goal_msg.pose.position.x = float(p[1])
        goal_msg.pose.position.y = float(p[0])


        goal_msg.pose.orientation.x = float(q[0])
        goal_msg.pose.orientation.y = float(q[1])
        goal_msg.pose.orientation.z = float(q[2])
        goal_msg.pose.orientation.w = float(q[3])

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        # self.car_state = "exec_goal"
        self.get_logger().info(f"Sending navigation goal: x={p[0]:.2f}, y={p[1]:.2f}")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available, cannot send goal!")
            # self.is_exploring = False
            return
def main(args=None):
    rclpy.init(args=args)
    node = GrabNode()
    rclpy.spin(node)
