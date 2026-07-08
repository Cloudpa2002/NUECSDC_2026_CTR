#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import json
from datetime import datetime
from geometry_msgs.msg import PoseStamped  # 导入PoseStamped消息类型

class TopicLogger(Node):
    def __init__(self):
        super().__init__('mavros_pose_logger')
        
        # ===== 配置参数 =====
        self.topic_name = "/mavros/local_position/pose"  # 目标话题名[6,7](@ref)
        self.msg_type = "geometry_msgs/msg/PoseStamped"  # 消息类型[3,7](@ref)
        self.output_dir = "~/ros2_local_position_pose_logs"  # 存储目录
        self.file_format = "json"  # 存储格式
        # ===================
        
        # 创建存储目录
        self.output_dir = os.path.expanduser(self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{self.output_dir}/mavros_pose_{timestamp}.{self.file_format}"
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            PoseStamped,  # 直接使用导入的消息类型
            self.topic_name,
            self.listener_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info(f"⏺️ 开始录制话题: {self.topic_name} → {self.filename}")

    def listener_callback(self, msg):
        try:
            # 根据格式选择存储方式
            with open(self.filename, 'a') as f:
                if self.file_format == "json":
                    # 提取关键数据[6,7](@ref)
                    data = {
                        'timestamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'position': {
                            'x': msg.pose.position.x,
                            'y': msg.pose.position.y,
                            'z': msg.pose.position.z
                        },
                        'orientation': {
                            'x': msg.pose.orientation.x,
                            'y': msg.pose.orientation.y,
                            'z': msg.pose.orientation.z,
                            'w': msg.pose.orientation.w
                        }
                    }
                    json.dump(data, f)
                    f.write('\n')
                else:  # txt格式
                    f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}: "
                            f"Pos({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})\n")
                    
        except Exception as e:
            self.get_logger().error(f"写入失败: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TopicLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 用户终止录制")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()