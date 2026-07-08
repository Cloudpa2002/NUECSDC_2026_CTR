# 查看无人机当前座标

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class UAVPositionPublisher(Node):
    def __init__(self):
        super().__init__('uav_position_publisher')

        self.current_pos = None

        # 订阅无人机当前位置（与原脚本一致） :contentReference[oaicite:0]{index=0}
        self.odom_sub = self.create_subscription(
            Odometry,
            "Odometry",
            self.odom_callback,
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            )
        )

        # 定时器：持续输出位置（10Hz）
        self.timer = self.create_timer(1, self.timer_callback)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])

    def timer_callback(self):
        if self.current_pos is not None:
            x, y, z = self.current_pos
            self.get_logger().info(f"当前位置坐标: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        else:
            self.get_logger().warning("尚未接收到位置数据...")

def main(args=None):
    rclpy.init(args=args)
    node = UAVPositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点已手动停止")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

