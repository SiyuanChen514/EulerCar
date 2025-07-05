import rclpy
import numpy as np

LIFT = 1
LAY = 0

def check_robot_ready_for_grab(explorer_node,world_target_loc):
        # 1. 判断是否到达目标点(位置和朝向)
        distance2target = np.sqrt((explorer_node.world_x - world_target_loc[0])**2 + (explorer_node.world_y - world_target_loc[1])**2)
        if distance2target > explorer_node.grab_threshold:
            return
        target_yaw = np.arctan2(world_target_loc[1], world_target_loc[0])
        yaw_diff = abs(target_yaw - explorer_node.world_yaw)
        if yaw_diff > np.pi:
            yaw_diff = 2*np.pi - yaw_diff
        if yaw_diff > explorer_node.orientation_threshold:
            return
        
        explorer_node.get_logger().info("robot is ready for grab!")
        return True

def grab_lift_box(explorer_node,world_box_loc):
        
        # 1. 坐标给nav2导航(完善)
        explorer_node.navigate_to(world_box_loc[0],world_box_loc[1])
        # 2. 判断是否到达目标点(位置和朝向)
        if  check_robot_ready_for_grab(explorer_node,world_box_loc)is not None:
            # 3. 向servo发送请求
            try:
                explorer_node.servo_client.wait_for_service(timeout_sec=explorer_node.wait_for_servo_timeout)
                request = explorer_node.servo_client(LIFT)
                future = explorer_node.servo_client.call(request) # 同步调用
                # 4. 等待抓取完成
                rclpy.spin_until_future_complete(explorer_node,future,timeout_sec=explorer_node.wait_for_grab_timeout)
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        explorer_node.get_logger().info("Grabbed the box with {response.message}")
                        return True
                    else:
                        explorer_node.get_logger().info("Failed to grab the box with {response.message}")
                        return
                else:
                    explorer_node.get_logger().info("Failed to grab the box with no response")
                    return
            except Exception as e:
                explorer_node.get_logger().error(f"Service call failed: {e}")
            return
    
def grab_lay_box(explorer_node,world_lay_corrds):

    explorer_node.navigate_to(world_lay_corrds)
    explorer_node.get_logger().info("Moving to drop off location!")
    # 识别位置(待补充)

    # 放下物体
    try:
        explorer_node.servo_client.wait_for_service(timeout_sec=explorer_node.wait_for_servo_timeout)
        request = explorer_node.servo_client(LAY)
        future = explorer_node.servo_client.call(request) # 同步调用
        rclpy.spin_until_future_complete(explorer_node,future,timeout_sec=explorer_node.wait_for_grab_timeout)
        if future.result() is not None:
            response = future.result()
            if response.success:
                explorer_node.get_logger().info("Placed the box with {response.message}")
            else:
                explorer_node.get_logger().info("Failed to place the box with {response.message}")
        else:
            explorer_node.get_logger().info("Failed to place the box with no response")
    except Exception as e:
        explorer_node.get_logger().error(f"Service call failed: {e}")
     