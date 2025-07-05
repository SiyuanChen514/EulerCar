import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

def transform2world(explorer_node,child_frame,coordinates):
        try:
            # 将给的坐标转换为pointStamped
            child_frame_point = PointStamped()
            child_frame_point.header.frame_id = child_frame
            child_frame_point.header.stamp = explorer_node.get_clock().now().to_msg()
            child_frame_point.point.x = coordinates[0]
            child_frame_point.point.y = coordinates[1]
            child_frame_point.point.z = coordinates[2]

            # 等待变换可用
            if not explorer_node.tf_buffer.can_transform(child_frame, explorer_node.world_frame, child_frame_point.header.stamp,timeout=rclpy.duration.Duration(seconds=explorer_node.wait_for_tf_timeout)):
                explorer_node.get_logger().warning("Transform from {child_frame} to {explorer_node.world_frame} not available!")
                return

            # 开始变换
            world_point = explorer_node.tf_buffer.transform(child_frame_point, explorer_node.world_frame)
            return [world_point.point.x, world_point.point.y, world_point.point.z]
        except Exception as e:
            explorer_node.get_logger().error(f"Coordinate transformation failed: {e}")
            return