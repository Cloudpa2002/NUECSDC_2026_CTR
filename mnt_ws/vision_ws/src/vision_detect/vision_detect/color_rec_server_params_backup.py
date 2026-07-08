#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
import cv2
import math
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange
from std_srvs.srv import SetBool  # 使用标准布尔服务类型
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter_service import ParameterService
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ColorDetectionServer(Node):
    def __init__(self, name):
        super().__init__(name)

        self._param_service = ParameterService(self)          
        self.param_client_deepseek = self.create_client(GetParameters, '/deepseek_node/get_parameters')
        # 当前状态存储
        self.current_pos = None


        self.odom_sub = self.create_subscription(
                Odometry,
                "/Odometry",
                self.odom_callback,
                rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        # 声明所有颜色阈值参数
        self.declare_color_parameters()
        # 声明处理参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('circularity_threshold', 0.85, ParameterDescriptor(description='Minimum circularity threshold (0-1)', type=ParameterType.PARAMETER_DOUBLE,floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])),
                ('min_contour_size', 200, ParameterDescriptor(description='Minimum contour size in pixels', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=300, step=1)])),
                ('erode_iterations', 2, ParameterDescriptor(description='Erosion iterations', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])),
                ('dilate_iterations', 2, ParameterDescriptor(description='Dilation iterations', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])),
                ('camera_index', 21, ParameterDescriptor(description='Camera index (usually 0 for built-in webcam)', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])),
                # td xuyaoshanchu
                # ('aim_color', 2, ParameterDescriptor(description='Target color to check (1=red, 2=green, 3=blue)', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=1, to_value=3, step=1)])),
                ('required_consecutive_frames', 5, ParameterDescriptor(description='Number of consecutive frames required for positive detection', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=1, to_value=30, step=1)]))
            ]
        )

        self.cv_bridge = CvBridge()

        # 创建服务
        self.check_color_service = self.create_service(
            SetBool, 
            'check_color_match', 
            self.check_color_match_callback
        )
        
        # 初始化对象信息
        self.objectX = 0
        self.objectY = 0
        self.object_color = ""
        self.detected_color_code = 0  # 0=未检测到, 1=红, 2=绿, 3=蓝
        self.consecutive_matches = 0  # 连续匹配的帧数
        self.consecutive_mismatches = 0  # 连续不匹配的帧数
        
        # 颜色代码映射
        self.color_code_map = {
            1: "red",
            2: "green",
            3: "blue",
            4: "orange",
        }
        
        # 循环检测是否从 deepseek 处获取颜色
        while rclpy.ok():
            self.aim_color = self.get_remote_param_deepseek('aim_color')
            if self.aim_color is not None and self.aim_color != 0:
                break
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"color = {self.aim_color}")

        # 打开摄像头
        self.camera_index = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera with index {self.camera_index}")
            raise RuntimeError("Camera open failed")
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度

        # 创建定时器处理摄像头帧
        self.timer = self.create_timer(0.033, self.process_camera_frame)  # ~30fps


    def odom_callback(self, msg):
        # 更新位置
        pos = msg.pose.pose.position
        self.current_pos = [pos.x, pos.y, pos.z]
        

    def declare_color_parameters(self):
        """声明所有颜色阈值参数"""
        # 公共参数描述符模板
        int_range_0_180 = [IntegerRange(from_value=0, to_value=180, step=1)]     # HSV-H范围
        int_range_0_255 = [IntegerRange(from_value=0, to_value=255, step=1)]     # HSV-S/V范围

        color_params = [
            # 红色参数 (双区间)
            ('red_h_low', 0, ParameterDescriptor(description='Red H low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('red_h_high', 10, ParameterDescriptor(description='Red H high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('red_h_low2', 170, ParameterDescriptor(description='Red H low2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('red_h_high2', 180, ParameterDescriptor(description='Red H high2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('red_s_low', 90, ParameterDescriptor(description='Red S low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('red_s_high', 255, ParameterDescriptor(description='Red S high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('red_v_low', 128, ParameterDescriptor(description='Red V low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('red_v_high', 255, ParameterDescriptor(description='Red V high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            
            # 橙色参数
            ('orange_h_low', 11, ParameterDescriptor(description='Orange H low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_180)),
            ('orange_h_high', 25, ParameterDescriptor(description='Orange H high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_180)),
            ('orange_s_low', 90, ParameterDescriptor(description='Orange S low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            ('orange_s_high', 255, ParameterDescriptor(description='Orange S high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            ('orange_v_low', 128, ParameterDescriptor(description='Orange V low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            ('orange_v_high', 255, ParameterDescriptor(description='Orange V high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),

            # 绿色参数
            ('green_h_low', 35, ParameterDescriptor(description='Green H low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('green_h_high', 85, ParameterDescriptor(description='Green H high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('green_s_low', 90, ParameterDescriptor(description='Green S low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('green_s_high', 255, ParameterDescriptor(description='Green S high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('green_v_low', 70, ParameterDescriptor(description='Green V low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('green_v_high', 255, ParameterDescriptor(description='Green V high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            
            # 蓝色参数
            ('blue_h_low', 100, ParameterDescriptor(description='Blue H low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('blue_h_high', 130, ParameterDescriptor(description='Blue H high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('blue_s_low', 90, ParameterDescriptor(description='Blue S low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('blue_s_high', 255, ParameterDescriptor(description='Blue S high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('blue_v_low', 70, ParameterDescriptor(description='Blue V low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('blue_v_high', 255, ParameterDescriptor(description='Blue V high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255))
        ]
        
        self.declare_parameters(namespace='', parameters=color_params)

    def get_color_thresholds(self):
        """从参数服务器获取当前颜色阈值配置"""
        # 获取所有颜色参数
        color_params = self.get_parameters([
            'red_h_low', 'red_h_high', 'red_h_low2', 'red_h_high2',
            'red_s_low', 'red_s_high', 'red_v_low', 'red_v_high',
            'orange_h_low', 'orange_h_high', 'orange_s_low', 'orange_s_high', 'orange_v_low', 'orange_v_high',
            'green_h_low', 'green_h_high', 'green_s_low', 'green_s_high', 'green_v_low', 'green_v_high',
            'blue_h_low', 'blue_h_high', 'blue_s_low', 'blue_s_high', 'blue_v_low', 'blue_v_high'
        ])
        
        # 构建颜色阈值字典
        return {
            "red": {
                "lower": np.array([color_params[0].value, color_params[4].value, color_params[6].value]),
                "upper": np.array([color_params[1].value, color_params[5].value, color_params[7].value]),
                "lower2": np.array([color_params[2].value, color_params[4].value, color_params[6].value]),
                "upper2": np.array([color_params[3].value, color_params[5].value, color_params[7].value])
            },
            "orange": {
                "lower": np.array([color_params[8].value, color_params[10].value, color_params[12].value]),
                "upper": np.array([color_params[9].value, color_params[11].value, color_params[13].value])
            },
            "green": {
                "lower": np.array([color_params[14].value, color_params[16].value, color_params[18].value]),
                "upper": np.array([color_params[15].value, color_params[17].value, color_params[19].value])
            },
            "blue": {
                "lower": np.array([color_params[20].value, color_params[22].value, color_params[24].value]),
                "upper": np.array([color_params[21].value, color_params[23].value, color_params[25].value])
            }
        }

    def is_circle(self, contour):
        """判断轮廓是否近似圆形"""
        circularity_threshold = self.get_parameter('circularity_threshold').value
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return False
            
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        return circularity > circularity_threshold

    def color_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        detected = False
        current_color_code = 0  # 0表示未检测到
        
        # 获取当前参数配置
        color_thresholds = self.get_color_thresholds()
        erode_iter = self.get_parameter('erode_iterations').value
        dilate_iter = self.get_parameter('dilate_iterations').value
        min_contour_size = self.get_parameter('min_contour_size').value

        # 检测每种颜色
        for color_name, thresholds in color_thresholds.items():
            # 红色需要特殊处理（两个区间）
            if color_name == "red":
                mask1 = cv2.inRange(hsv_img, thresholds["lower"], thresholds["upper"])
                mask2 = cv2.inRange(hsv_img, thresholds["lower2"], thresholds["upper2"])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_img, thresholds["lower"], thresholds["upper"])
            
            # 形态学操作
            if erode_iter > 0:
                mask = cv2.erode(mask, None, iterations=erode_iter)
            if dilate_iter > 0:
                mask = cv2.dilate(mask, None, iterations=dilate_iter)
            
            # 轮廓检测
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            for cnt in contours:
                if cnt.shape[0] < min_contour_size:
                    continue
                
                if not self.is_circle(cnt):
                    continue

                (x, y, w, h) = cv2.boundingRect(cnt)
                center_x = int(x + w/2)
                center_y = int(y + h/2)
                radius = int(math.sqrt(w*w + h*h)/2)
                
                # 绘制检测结果
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.circle(image, (center_x, center_y), radius, (255, 0, 0), 2)
                cv2.putText(image, color_name.upper(), (center_x - 20, center_y - radius - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # 更新检测结果
                if not detected:
                    self.objectX = center_x
                    self.objectY = center_y
                    self.object_color = color_name
                    # 设置当前颜色代码
                    if color_name == "red":
                        current_color_code = 4
                    elif color_name == "green":
                        current_color_code = 2
                    elif color_name == "blue":
                        current_color_code = 3
                    elif color_name == "orange":
                        current_color_code = 1
                    detected = True

        # 更新连续匹配计数
        required_frames = self.get_parameter('required_consecutive_frames').value
        
        if detected and current_color_code == self.aim_color:
            self.consecutive_matches += 1
            self.consecutive_mismatches = 0
        else:
            self.consecutive_matches = 0
            self.consecutive_mismatches += 1
        
        # 确保连续匹配计数不超过要求帧数
        if self.consecutive_matches > required_frames:
            self.consecutive_matches = required_frames
        
        self.detected_color_code = current_color_code
        
        cv2.imshow("Color Detection", image)
        cv2.waitKey(1)

    def process_camera_frame(self):
        """从摄像头捕获并处理帧"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera")
            return
            
        self.color_detect(frame)

    #获取目标颜色
    def get_remote_param_deepseek(self,param):
        req = GetParameters.Request()
        req.names = [param]  # 请求参数名
        future = self.param_client_deepseek.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        # 检查服务调用结果
        if future.result() is None:
            self.get_logger().warn(f"Service call to get parameter {param} failed or timed out")
            return None

        # 检查返回值是否包含参数
        if not future.result().values:
            self.get_logger().warn(f"Parameter {param} not found or empty")
            return None
        return future.result().values[0].integer_value  # 假设返回的是一个列表，取第一个值
        

    def check_color_match_callback(self, request, response):
        """
        服务回调函数，检查当前检测到的颜色是否符合目标颜色
        request.data: 服务请求的布尔值（未使用）
        response.success: 返回是否匹配
        response.message: 返回详细信息
        """
        required_frames = self.get_parameter('required_consecutive_frames').value

        if self.consecutive_matches >= required_frames:
            response.success = True
            color_name = self.color_code_map.get(self.aim_color, "unknown")
            response.message = f"Detected color {self.object_color} matches target color {color_name} for {self.consecutive_matches} consecutive frames"
            self.get_logger().info('true')
        else:
            response.success = False
            if self.detected_color_code == 0:
                response.message = "No color detected"
            else:
                detected_name = self.color_code_map.get(self.detected_color_code, "unknown")
                target_name = self.color_code_map.get(self.aim_color, "unknown")
                response.message = f"Detected color {detected_name} does not match target color {target_name} (consecutive matches: {self.consecutive_matches}/{required_frames})"
            
        self.get_logger().info(response.message)
        return response

    def __del__(self):
        """析构函数，释放摄像头资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionServer("color_detection_server")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
