# 发送目标点

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPointSender(Node):
    def __init__(self):
        super().__init__('goal_point_sender')

        # 发布目标点（给路径规划模块使用）
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # 目标点参数，可按需修改
        self.target_x = 3.9
        self.target_y = 4.05
        self.target_z = 1.0

        # 为了确保规划模块能收到，连续发送几次目标点
        self.publish_count = 0
        self.max_publish_count = 1
        self.timer = self.create_timer(0.5, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world'

        goal.pose.position.x = self.target_x
        goal.pose.position.y = self.target_y
        goal.pose.position.z = self.target_z

        # 不关心朝向时，保持单位四元数即可
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal)
        self.publish_count += 1

        self.get_logger().info(
            f'已发送目标点: x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f} '
            f'({self.publish_count}/{self.max_publish_count})'
        )

        # 发送若干次后自动停止，不控制无人机运动
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info('目标点发送完成，节点即将退出。')
            self.destroy_timer(self.timer)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPointSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户手动中断')

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

