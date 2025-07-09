import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

def transform2world(explorer_node,child_frame,coordinates):
        """
        将子坐标系中的坐标转换为世界坐标系中的坐标

        参数:
            explorer_node: ExplorerNode实例，包含tf_buffer、world_frame等属性
            child_frame (str): 源坐标系名称，如"camera_link"
            coordinates (list): 长度为3的列表，包含[x, y, z]坐标值

        返回值:
            list: 转换后的世界坐标系坐标[x, y, z]，转换失败时返回None
        """
        try:
            # 将给的坐标转换为pointStamped
            child_frame_point = PointStamped()
            child_frame_point.header.frame_id = child_frame
            child_frame_point.header.stamp = explorer_node.get_clock().now().to_msg()
            child_frame_point.point.x = coordinates[0]
            child_frame_point.point.y = coordinates[1]
            child_frame_point.point.z = coordinates[2]

            # 等待变换可用
            if not explorer_node.tf_buffer.can_transform(child_frame, 
                                                        explorer_node.world_frame, 
                                                        child_frame_point.header.stamp,
                                                        timeout=rclpy.duration.Duration(seconds=explorer_node.wait_for_tf_timeout)):
                explorer_node.get_logger().info("Transform from {child_frame} to {explorer_node.world_frame} not available!")
                return

            # 开始变换
            world_point = explorer_node.tf_buffer.transform(child_frame_point,
                                                            explorer_node.world_frame,
                                                            timeout=rclpy.duration.Duration(seconds=explorer_node.wait_for_tf_timeout))
            explorer_node.getlogger().info(f'Transformed point from {child_frame} to {explorer_node.world_frame}: '
                f'x={world_point.point.x:.3f}, y={world_point.point.y:.3f}, z={world_point.point.z:.3f}')
            return [world_point.point.x, world_point.point.y, world_point.point.z]
        except Exception as e:
            explorer_node.get_logger().error(f"Coordinate transformation failed: {e}")
            return