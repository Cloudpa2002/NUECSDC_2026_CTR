import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import shutil  # 用于删除目录内容
import cv2  # 用于摄像头操作
from datetime import datetime

class PhotoCaptureNode(Node):
    def __init__(self):
        super().__init__('photo_capture_node')

        # 订阅 /Odometry 话题
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        # 初始化变量
        self.photo_dir = os.path.expanduser("~/photos")
        self.clear_photo_directory()  # 清空照片目录
        self.current_position = None
        self.photo_count = 0
        self.max_photos = 100

        # 打开摄像头
        self.cap = cv2.VideoCapture(21)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头，请检查设备连接。")

        # 设置摄像头分辨率为 640 x 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.get_logger().info("PhotoCaptureNode 已启动，等待位置信息...")

    def clear_photo_directory(self):
        """清空照片保存目录"""
        if os.path.exists(self.photo_dir):
            try:
                shutil.rmtree(self.photo_dir)  # 删除整个目录及其内容
                self.get_logger().info(f"已清空照片目录: {self.photo_dir}")
            except Exception as e:
                self.get_logger().error(f"清空照片目录失败: {e}")
        os.makedirs(self.photo_dir, exist_ok=True)  # 重新创建目录

    def odom_callback(self, msg):
        # 从 /Odometry 消息中提取位置信息
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y, position.z)

        # 如果位置信息有效且未达到最大拍照数量，则拍照
        if self.current_position is not None and self.photo_count < self.max_photos:
            self.capture_photo()

    def capture_photo(self):
        # 生成照片文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        x, y, z = self.current_position
        photo_name = f"photo_xyz_{x:.2f}_{y:.2f}_{z:.2f}_{timestamp}.jpg"
        photo_path = os.path.join(self.photo_dir, photo_name)

        try:
            # 从摄像头读取一帧图像
            ret, frame = self.cap.read()
            if not ret:
                raise RuntimeError("无法从摄像头读取图像。")

            # 保存图像到指定路径
            cv2.imwrite(photo_path, frame)
            self.photo_count += 1
            self.get_logger().info(f"照片已保存到: {photo_path} ({self.photo_count}/{self.max_photos})")
        except Exception as e:
            self.get_logger().error(f"拍照失败: {e}")

        # 如果已拍满 100 张照片，停止节点
        if self.photo_count >= self.max_photos:
            self.get_logger().info("已拍摄 100 张照片，节点即将退出...")
            rclpy.shutdown()

    def destroy_node(self):
        """在节点销毁时释放摄像头资源"""
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("摄像头资源已释放")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
