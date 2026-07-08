#!/usr/bin/env python3
# 识别圆形并发布圆心相对于相机/无人机当前位置的 xy 偏移
# Ubuntu 22.04 + ROS2 Humble + OpenCV

import cv2
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point

# ================== 需要根据实际相机修改的参数 ==================

CAMERA_INDEX = 21  # 相机设备编号

# 圆的真实直径，单位：米
D_REAL = 0.40

# 相机内参，单位：像素
fx = 732.22  # 相机 x 方向焦距
fy = 730.01  # 相机 y 方向焦距
cx_camera = 623.39  # 相机主点横向像素坐标
cy_camera = 445.52  # 相机主点纵向像素坐标

# 发送给 C++ 控制节点的话题名。C++ 文件中订阅同名话题。
CENTER_TOPIC = "/vision_circle_center"

# 如果相机坐标轴与无人机世界坐标轴方向相反，可在这里调整符号。
# 当前实现严格按“目标点 = 当前世界坐标 + 圆心 x/y 坐标”发布。
OFFSET_X_SIGN = 1.0
OFFSET_Y_SIGN = 1.0

# ================================================================


class VisionCircleNode(Node):
    def __init__(self):
        super().__init__("vision_circle_node")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.center_pub = self.create_publisher(Point, CENTER_TOPIC, qos)

        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)

        if not self.cap.isOpened():
            raise RuntimeError("cannot open camera")

        self.get_logger().info(f"vision circle node started, publishing {CENTER_TOPIC}")

    def detect_best_circle(self, frame):
        if len(frame.shape) == 2:
            gray = frame
            out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            out = frame.copy()

        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 180)

        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        best = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 2000:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri < 100:
                continue

            if len(cnt) < 5:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = w / float(h)
            if ratio < 0.75 or ratio > 1.25:
                continue

            circularity = 4 * math.pi * area / (peri * peri)
            if circularity < 0.75:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (u, v), (axis1, axis2), angle = ellipse

            major_axis = max(axis1, axis2)
            minor_axis = min(axis1, axis2)

            # 用长短轴平均值估算圆的像素直径
            d_pixel = (major_axis + minor_axis) / 2.0
            if d_pixel <= 0:
                continue

            # 深度估算，单位：米
            z = fx * D_REAL / d_pixel

            # 圆心相对于相机坐标系的坐标，单位：米
            x_cam = (u - cx_camera) * z / fx
            y_cam = (v - cy_camera) * z / fy

            candidate = {
                "area": area,
                "ellipse": ellipse,
                "bbox": (x, y, w, h),
                "u": u,
                "v": v,
                "d_pixel": d_pixel,
                "x_cam": x_cam,
                "y_cam": y_cam,
                "z": z,
            }

            # 多个圆候选时，选择面积最大的一个，避免一帧内发布多个目标
            if best is None or candidate["area"] > best["area"]:
                best = candidate

        return best, gray, edges, out

    def publish_circle_center(self, circle):
        msg = Point()
        msg.x = OFFSET_X_SIGN * circle["x_cam"]
        msg.y = OFFSET_Y_SIGN * circle["y_cam"]
        msg.z = circle["z"]
        self.center_pub.publish(msg)

        print(
            f"publish {CENTER_TOPIC}: "
            f"x={msg.x:.4f}m, y={msg.y:.4f}m, z={msg.z:.4f}m; "
            f"pixel_center: u={circle['u']:.2f}, v={circle['v']:.2f}, "
            f"D_pixel={circle['d_pixel']:.2f}px"
        )

    def draw_circle(self, out, circle):
        x, y, w, h = circle["bbox"]
        u = circle["u"]
        v = circle["v"]
        x_cam = circle["x_cam"]
        y_cam = circle["y_cam"]
        z = circle["z"]
        d_pixel = circle["d_pixel"]

        cv2.ellipse(out, circle["ellipse"], (0, 255, 0), 2)
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(out, (int(u), int(v)), 5, (0, 0, 255), -1)

        cv2.putText(
            out,
            f"X={x_cam:.3f}m Y={y_cam:.3f}m Z={z:.3f}m",
            (int(u) + 10, int(v) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )

        cv2.putText(
            out,
            f"D_pixel={d_pixel:.1f}px",
            (int(u) + 10, int(v) + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )

    def run(self):
        try:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("cannot read frame")
                    break

                circle, gray, edges, out = self.detect_best_circle(frame)

                if circle is not None:
                    self.publish_circle_center(circle)
                    self.draw_circle(out, circle)

                cv2.imshow("gray", gray)
                cv2.imshow("edges", edges)
                cv2.imshow("result", out)

                rclpy.spin_once(self, timeout_sec=0.001)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
        finally:
            self.cap.release()
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VisionCircleNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
