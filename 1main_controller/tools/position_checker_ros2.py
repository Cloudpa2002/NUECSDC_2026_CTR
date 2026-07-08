import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

def wait_until_position_reached(node,
    target_position, 
    position_threshold=0.05, 
    velocity_threshold=0.20, 
    timeout_sec=180.0,
    odom_topic="/Odometry",
    direction=None
):
    """
    阻塞式等待无人机到达指定位置并停稳
    参数：
        target_position: 目标位置 [x, y, z] (list或np.array)
        position_threshold: 位置判定阈值（米）
        velocity_threshold: 速度判定阈值（m/s）
        timeout_sec: 超时时间（秒）
        odom_topic: 里程计话题
    返回：
        bool: 是否在超时前成功到达
    """
    class PositionChecker(Node):
        def __init__(self):
            super().__init__('position_checker_temp_node')
            self.target_pos = np.array(target_position)
            self.position_th = position_threshold
            self.velocity_th = velocity_threshold
            self.direction = direction  # 方向参数，None表示检查所有方向，'x', 'y', 'z'表示只检查对应方向   
            
            # 当前状态存储
            self.current_pos = None
            self.current_vel = np.zeros(3)
            self.odom_sub = self.create_subscription(
                Odometry,
                odom_topic,
                self.odom_callback,
                rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
            )

        def odom_callback(self, msg):
            # 更新位置
            pos = msg.pose.pose.position
            self.current_pos = np.array([pos.x, pos.y, pos.z])
            
            # 更新速度
            vel = msg.twist.twist.linear
            self.current_vel = np.array([vel.x, vel.y, vel.z])

        def is_target_reached(self):
            if self.direction is None:
                # 记录当前位置信息
                if self.current_pos is None:
                    return False
                    
                # 计算位置距离（只计算 x 和 y）
                pos_dist = np.linalg.norm(self.current_pos - self.target_pos)
                
                self.get_logger().info(
                    f"Distance: {pos_dist:.2f}m ，current_pos = {self.current_pos},",
                    throttle_duration_sec=1.0
                )
                # self.get_logger().info(f"Current position: {self.current_pos}")
                return (pos_dist < self.position_th)
            
            if self.direction == "x":
                # 只检查 x 方向
                if self.current_pos is None:
                    return False
                    
                pos_dist = abs(self.current_pos[0] - self.target_pos[0])
                self.get_logger().info(
                    f"X Distance: {pos_dist:.2f}m ，current_pos = {self.current_pos}",
                    throttle_duration_sec=1.0
                )
                return (pos_dist < self.position_th)
            if self.direction == "y":
                # 只检查 y 方向
                if self.current_pos is None:
                    return False
                    
                pos_dist = abs(self.current_pos[1] - self.target_pos[1])
                self.get_logger().info(
                    f"Y Distance: {pos_dist:.2f}m ，current_pos = {self.current_pos}",
                    throttle_duration_sec=1.0
                )
                return (pos_dist < self.position_th)
            if self.direction == "z":
                # 只检查 z 方向
                if self.current_pos is None:
                    return False
                    
                pos_dist = abs(self.current_pos[2] - self.target_pos[2])
                self.get_logger().info(
                    f"Z Distance: {pos_dist:.2f}m ，current_pos = {self.current_pos}",
                    throttle_duration_sec=1.0
                )
                return (pos_dist < self.position_th)
        

    # 初始化ROS2上下文
    checker = PositionChecker()
    start_time = checker.get_clock().now()
    
    try:
        while rclpy.ok():
            # 检查超时
            if (checker.get_clock().now() - start_time) > Duration(seconds=timeout_sec):
                checker.get_logger().warn("Timeout while waiting for position!")
                return False
                
            # 单次轮询
            rclpy.spin_once(checker, timeout_sec=0.1)
            
            if checker.is_target_reached():
                checker.get_logger().info("Target position reached with stable state!")
                return True
                
    finally:
        checker.destroy_node()


# 使用示例
if __name__ == "__main__":
    success = wait_until_position_reached(
        target_position=[5.0, 3.0, 2.0],
        position_threshold=0.15,
        velocity_threshold=0.05,
        timeout_sec=60.0
    )
    print("Mission result:", success)
