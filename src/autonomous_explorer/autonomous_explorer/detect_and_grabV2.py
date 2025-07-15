import rclpy
import numpy as np
from .img_detectV2 import *
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time # Import time for delays
# from geometry_msgs.msg import PoseStamped
# from ServoControl.srv import ServoControl


class GrabNode(Node):
    
    def __init__(self):

        super().__init__('grab_node')

        self.source = 2        
        # 相机内参
        self.img_width = 1280
        self.img_height = 480
        self.cap = cv2.VideoCapture(self.source)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)   # 宽度
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 高度




        self.beginGrab = False
        self.grabbed = False

        self.last_yaw_error = 0.0

        self.max_vel = 0.02   # 20mm/s
        self.max_angular = 0.2  # 0.2rad/s

        self.kp_ = 0.02
        self.ki_ = 0.05

        # self.client = self.create_client(ServoControl, 'ServoService')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('服务 ServoService 不可用，等待中...')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  
        
        self.DetectBox_timer = self.create_timer(
            0.1, self.detectCallback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        
        # self.grabTimer = self.create_timer(
        #     0.1, self.grabCallback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
    # def send_request(self,req_):
    #     if not self.client.service_is_ready():
    #         self.get_logger().warn('服务未就绪')
    #         return
    #     # 创建请求
    #     req = ServoControl.Request()
    #     # 设置请求数据（示例：8字节命令）
    #     req.command = req_
        
    #     # 发送异步请求
    #     future = self.client.call_async(req)
        
    #     # 设置回调函数处理响应
    #     future.add_done_callback(self.handle_response)
    # def handle_response(self, future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info(f"发送成功: {response.res}")
    #     except Exception as e:
    #         self.get_logger().error(f"发送失败: {e}")
    def publish_velocity_command(self, linear_x, angular_z):
        """
        Publishes a Twist message with specified linear and angular velocities.
        """
        twist_msg = Twist()

        # Set linear velocity (forward/backward motion)
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        # Set angular velocity (rotational motion)
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular_z)

        self.publisher_.publish(twist_msg)

    def detectCallback(self):
        if not self.cap.isOpened():
            print("无法打开视频设备")
            return
        ret, frame = self.cap.read()
        if not ret:
                print("无法获取帧")
                return 
        
        detected_,disparity = detect_green_objects(frame,self.img_width,self.img_height)
        if detected_ and not self.grabbed:
            self.beginGrab = True
        else:
            return
        if self.beginGrab:
            self.adjust_yaw(disparity)
    def adjust_yaw(self,disparity):

        yaw_error = (disparity[0]+disparity[1]+disparity[2]+disparity[3])/2 - self.img_width/2
        print(f"disparity: {disparity}")
        print(f"yaw_error: {yaw_error}")

        if abs(disparity[1] - self.img_width/2)<5 and disparity[2]<5:
            v_liner = 0
            z_angular = 0
            self.beginGrab = False
            self.grabbed = True
            self.publish_velocity_command(v_liner, z_angular)
            # # req = self.pack_data(self.v_left,self.v_right)
            # # # self.send_request(req)
            # print(f"左轮速度: {self.v_left}, 右轮速度: {self.v_right}")
            # print(f"结果: {[hex(b) for b in req]}")
        elif abs(yaw_error) > 30:
            z_angular = -min(max(yaw_error * self.kp_ , -self.max_angular),self.max_angular)
            # self.v_right = -self.v_left
            # self.last_yaw_error = yaw_error
            # req = self.pack_data(self.v_left,self.v_right)
            # print(f"左轮速度: {self.v_left}, 右轮速度: {self.v_right}")
            # print(f"结果: {[hex(b) for b in req]}")
            # self.send_request(req)
            self.publish_velocity_command(v_liner, z_angular)

        elif abs(yaw_error) <= 30:
            print("gogogogogogogogoggogo")
            # self.v_left = self.max_vel
            # self.v_right = self.max_vel
            # req = self.pack_data(self.v_left,self.v_right)
            # print(f"左轮速度: {self.v_left}, 右轮速度: {self.v_right}")
            # print(f"结果: {[hex(b) for b in req]}")
            # # self.send_request(req)
            v_liner = self.max_vel
            z_angular = 0
            self.publish_velocity_command(v_liner, z_angular)
    # def pack_data(self,v1, v2):
    #     """
    #     将整数v1和v2转换为两字节数据,并按照指定规则填充到uint8[8]数组中
        
    #     参数:
    #     - v1: 第一个整数，范围：-32768到32767
    #     - v2: 第二个整数，范围：-32768到32767
        
    #     返回:
    #     - 填充好的uint8[8]数组
    #     """
    #     # 将v1转换为两字节（大端序）
    #     v1 = int(v1)
    #     v2 = int(v2)

    #     v1_high = (v1 >> 8) & 0xFF  # 高字节
    #     v1_low = v1 & 0xFF          # 低字节
        
    #     # 将v2转换为两字节（大端序）
    #     v2_high = (v2 >> 8) & 0xFF  # 高字节
    #     v2_low = v2 & 0xFF          # 低字节
        
    #     # 填充到uint8[8]数组中
    #     data = [
    #         v1_high,  # 第1位：v1的高字节
    #         v1_low,   # 第2位：v1的低字节
    #         v2_high,  # 第3位：v2的高字节
    #         v2_low,   # 第4位：v2的低字节
    #         0,        # 第5位：0
    #         0,        # 第6位：0
    #         0x01,     # 第7位：0x01
    #         0         # 第8位：0
    #     ]
        
    #     return data
             
        


      

        
def main(args=None):
    rclpy.init(args=args)
    node = GrabNode()
    rclpy.spin(node)