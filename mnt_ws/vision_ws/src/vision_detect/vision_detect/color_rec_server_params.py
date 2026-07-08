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

class ColorDetectionServer(Node):
    def __init__(self, name):
        super().__init__(name)

        self._param_service = ParameterService(self)          
        self.param_client_deepseek = self.create_client(GetParameters, '/deepseek_node/get_parameters')
        self.current_pos = None

        self.pos_extected = None  # 期望位置
        self.dx_mm_trans = None
        self.dy_mm_trans = None

        # 圆环距离计算
        self.template_path = '/home/elf/mnt_ws/vision_ws/src/vision_detect/vision_detect/trcir.png'  
        # 读取模板图片
        template_img_raw = cv2.imread(self.template_path)
        if template_img_raw is None:
            raise FileNotFoundError(f"Template image not found. Ensure {self.template_path} exists.")
        self.template_circles = self.extract_circles_from_image(template_img_raw)

        # 订阅当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            "Odometry",
            self.odom_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        # 声明所有颜色阈值参数
        self.declare_color_parameters()
        # 声明处理参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('circularity_threshold', 0.70, ParameterDescriptor(description='Minimum circularity threshold (0-1)', type=ParameterType.PARAMETER_DOUBLE,floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])),
                ('min_contour_size', 200, ParameterDescriptor(description='Minimum contour size in pixels', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=300, step=1)])),
                ('erode_iterations', 2, ParameterDescriptor(description='Erosion iterations', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])),
                ('dilate_iterations', 2, ParameterDescriptor(description='Dilation iterations', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])),
                ('camera_index', 21, ParameterDescriptor(description='Camera index (usually 0 for built-in webcam)', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])),
                # td xuyaoshanchu
                # ('aim_color', 1, ParameterDescriptor(description='Target color to check (1=orange, 2=green, 3=blue)', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=1, to_value=3, step=1)])),
                ('required_consecutive_frames', 5, ParameterDescriptor(description='Number of consecutive frames required for positive detection', type=ParameterType.PARAMETER_INTEGER,integer_range=[IntegerRange(from_value=1, to_value=30, step=1)]))
            ]
        )

        self.cv_bridge = CvBridge()

        # 创建判断颜色匹配的服务
        self.check_color_service = self.create_service(
            SetBool, 
            'check_color_match', 
            self.check_color_match_callback
        )
        # 创建发送预期位置的服务
        self.get_distance_error_service = self.create_service(
            SetBool, 
            'get_distance_error_values', 
            self.get_distance_error_callback
        )

        # 初始化对象信息
        self.objectX = 0
        self.objectY = 0
        self.object_color = ""
        self.detected_color_code = 0  # 0=未检测到, 1=橙, 2=绿, 3=蓝
        self.consecutive_matches = 0  # 连续匹配的帧数
        self.consecutive_mismatches = 0  # 连续不匹配的帧数
        
        # 颜色代码映射
        self.color_code_map = {
            # 4: "red",
            2: "green",
            3: "blue",
            1: "orange",
        }
        
        # 循环检测是否从 deepseek 处获取颜色 td
        while rclpy.ok():
            # self.aim_color = self.get_parameter('aim_color').value
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
        self.current_pos = np.array([pos.x, pos.y, pos.z])
        # self.get_logger().info(f"Current position updated: {self.current_pos}")

    # 服务回调函数
    def get_distance_error_callback(self, request, response):
        """
        服务回调函数，返回误差值 dx_mm 和 dy_mm
        request.data: 服务请求的布尔值
        response.success: 返回是否成功
        response.message: 返回误差值信息
        """
        if request.data:  # 如果请求为 true
            if hasattr(self, 'dx_mm') and hasattr(self, 'dy_mm') and self.dx_mm is not None and self.dy_mm is not None:
                response.success = True
                response.message = f"pos_extected: {self.pos_extected}"

                print(f"已调用服务")
                print(f"dx_mm:{self.dx_mm}  dy_mm:{self.dy_mm}")
                print(f"dx_mm_trans:{self.dx_mm_trans}  dy_mm_trans:{self.dy_mm_trans}")
                print(f"current_pos:{self.current_pos}")
                print(f"pos_extected:{self.pos_extected}")
                
            else:
                response.success = False
                response.message = "Error values not available. Ensure detection has been performed."
        else:
            response.success = False
            response.message = "Request data is false. No error values returned."


        self.get_logger().info(response.message)
        return response

    def declare_color_parameters(self):
        """声明所有颜色阈值参数"""
        # 公共参数描述符模板
        int_range_0_180 = [IntegerRange(from_value=0, to_value=180, step=1)]     # HSV-H范围
        int_range_0_255 = [IntegerRange(from_value=0, to_value=255, step=1)]     # HSV-S/V范围

        color_params = [
            # 红色参数 (双区间)
            # ('red_h_low', 0, ParameterDescriptor(description='Red H low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            # ('red_h_high', 25, ParameterDescriptor(description='Red H high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            # ('red_h_low2', 160, ParameterDescriptor(description='Red H low2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            # ('red_h_high2', 180, ParameterDescriptor(description='Red H high2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            # ('red_s_low', 90, ParameterDescriptor(description='Red S low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            # ('red_s_high', 255, ParameterDescriptor(description='Red S high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            # ('red_v_low', 100, ParameterDescriptor(description='Red V low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            # ('red_v_high', 255, ParameterDescriptor(description='Red V high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),

            # （橙色参数）
            ('orange_h_low', 0, ParameterDescriptor(description='orange H low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('orange_h_high', 25, ParameterDescriptor(description='orange H high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('orange_h_low2', 160, ParameterDescriptor(description='orange H low2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('orange_h_high2', 180, ParameterDescriptor(description='orange H high2', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_180)),
            ('orange_s_low', 90, ParameterDescriptor(description='orange S low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('orange_s_high', 255, ParameterDescriptor(description='orange S high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('orange_v_low', 100, ParameterDescriptor(description='orange V low', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            ('orange_v_high', 255, ParameterDescriptor(description='orange V high', type=ParameterType.PARAMETER_INTEGER,integer_range=int_range_0_255)),
            
            # 橙色参数
            # ('orange_h_low', 0, ParameterDescriptor(description='Orange H low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_180)),
            # ('orange_h_high', 0, ParameterDescriptor(description='Orange H high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_180)),
            # ('orange_s_low', 100, ParameterDescriptor(description='Orange S low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            # ('orange_s_high', 255, ParameterDescriptor(description='Orange S high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            # ('orange_v_low', 100, ParameterDescriptor(description='Orange V low', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),
            # ('orange_v_high', 255, ParameterDescriptor(description='Orange V high', type=ParameterType.PARAMETER_INTEGER, integer_range=int_range_0_255)),

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
            # 'red_h_low', 'red_h_high', 'red_h_low2', 'red_h_high2',
            # 'red_s_low', 'red_s_high', 'red_v_low', 'red_v_high',
            'orange_h_low', 'orange_h_high', 'orange_h_low2', 'orange_h_high2',
            'orange_s_low', 'orange_s_high', 'orange_v_low', 'orange_v_high',
            # 'orange_h_low', 'orange_h_high', 'orange_s_low', 'orange_s_high', 'orange_v_low', 'orange_v_high',
            'green_h_low', 'green_h_high', 'green_s_low', 'green_s_high', 'green_v_low', 'green_v_high',
            'blue_h_low', 'blue_h_high', 'blue_s_low', 'blue_s_high', 'blue_v_low', 'blue_v_high'
        ])
        
        # 构建颜色阈值字典
        return {
            "orange": {
                "lower": np.array([color_params[0].value, color_params[4].value, color_params[6].value]),
                "upper": np.array([color_params[1].value, color_params[5].value, color_params[7].value]),
                "lower2": np.array([color_params[2].value, color_params[4].value, color_params[6].value]),
                "upper2": np.array([color_params[3].value, color_params[5].value, color_params[7].value])
            },
            # "orange": {
            #     "lower": np.array([color_params[8].value, color_params[10].value, color_params[12].value]),
            #     "upper": np.array([color_params[9].value, color_params[11].value, color_params[13].value])
            # },
            "green": {
                "lower": np.array([color_params[8].value, color_params[10].value, color_params[12].value]),
                "upper": np.array([color_params[9].value, color_params[11].value, color_params[13].value])
            },
            "blue": {
                "lower": np.array([color_params[14].value, color_params[16].value, color_params[18].value]),
                "upper": np.array([color_params[15].value, color_params[17].value, color_params[19].value])
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

    def is_close_center(self,c1, c2, threshold=10):
        """判断两个圆心距离是否接近"""
        return np.hypot(c1[0] - c2[0], c1[1] - c2[1]) < threshold

    def extract_circles_from_image(self,image):
        """从图像中提取圆环，返回圆心和半径"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 11, 2)
        kernel = np.ones((3, 3), np.uint8)
        morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, hierarchy = cv2.findContours(morphed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        rings = []
        if hierarchy is None:
            return rings
        for i, cnt in enumerate(contours):
            if hierarchy[0][i][3] == -1 or cv2.contourArea(cnt) < 50:
                continue
            parent_idx = hierarchy[0][i][3]
            outer_cnt = contours[parent_idx]
            area_outer = cv2.contourArea(outer_cnt)
            area_inner = cv2.contourArea(cnt)
            if area_inner < 0.1 * area_outer or area_outer < 50:
                continue
            (x, y), r = cv2.minEnclosingCircle(outer_cnt)
            area_circle = np.pi * r * r
            circularity = area_outer / (area_circle + 1e-6)
            if r < 10 or circularity < 0.5:
                continue
            rings.append((int(x), int(y), int(r)))
        return rings

    def match_circles(self,template_circles, test_circles, threshold_center=10, threshold_radius=10):
        """判断检测到的圆环组是否与模板结构匹配"""
        if len(test_circles) < 3 or len(template_circles) < 3:
            return False
        template_sorted = sorted(template_circles, key=lambda c: c[2])
        template_rs = sorted([c[2] for c in template_sorted[:3]])
        for i in range(len(test_circles)):
            cx, cy, _ = test_circles[i]
            group = [test_circles[i]]
            for j in range(len(test_circles)):
                if i == j:
                    continue
                if self.is_close_center((cx, cy), (test_circles[j][0], test_circles[j][1]), threshold_center):
                    group.append(test_circles[j])
            if len(group) >= 3:
                group = sorted(group, key=lambda c: c[2])[:3]
                test_rs = sorted([c[2] for c in group])
                ratios_template = [template_rs[1] / template_rs[0], template_rs[2] / template_rs[0]]
                ratios_test = [test_rs[1] / test_rs[0], test_rs[2] / test_rs[0]]
                if all(abs(r1 - r2) < 0.2 for r1, r2 in zip(ratios_template, ratios_test)):
                    if all(abs(r1 - r2) < threshold_radius for r1, r2 in zip(template_rs, test_rs)):
                        return True
        return False

    def draw_center_x_symbol(self,img, center, color=(0,0,255), size=14, thickness=2):
        """在指定位置画红色X"""
        x, y = center
        cv2.line(img, (x-size, y-size), (x+size, y+size), color, thickness)
        cv2.line(img, (x-size, y+size), (x+size, y-size), color, thickness)

    def process_frame(self,frame, template_circles):
        # 旋转180度
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]
        x0, y0 = int(w * 0.1), int(h * 0)
        x1, y1 = int(w * 0.9), int(h * 0.7)
        roi_frame = frame[y0:y1, x0:x1].copy()
        circles = self.extract_circles_from_image(roi_frame)
        found = False
        group = []
        for i in range(len(circles)):
            cx, cy, _ = circles[i]
            temp_group = [circles[i]]
            for j in range(len(circles)):
                if i == j:
                    continue
                if self.is_close_center((cx, cy), (circles[j][0], circles[j][1])):
                    temp_group.append(circles[j])
            if len(temp_group) >= 3:
                temp_group = sorted(temp_group, key=lambda c: c[2])[:3]
                group = temp_group
                found = True
                break
        match_template = False
        mm_per_px = None
        dx_mm = dy_mm = None
        diameters = None
        output = frame.copy()
        cv2.rectangle(output, (x0, y0), (x1, y1), (255, 0, 0), 2)
        cx_img, cy_img = w // 2, h // 2
        self.draw_center_x_symbol(output, (cx_img, cy_img), color=(0,0,255), size=14, thickness=2)
        if found:
            match_template = self.match_circles(template_circles, group)
            actual_diams = [110, 250, 400]
            pixel_diams = [2*r for (_, _, r) in sorted(group, key=lambda c: c[2])]
            scale_list = [real/pixel for real, pixel in zip(actual_diams, pixel_diams)]
            mm_per_px = sum(scale_list) / len(scale_list)
            diameters = pixel_diams
            xc, yc = x0 + group[0][0], y0 + group[0][1]
            dx = xc - cx_img
            dy = yc - cy_img
            self.dx_mm = dx * mm_per_px
            self.dy_mm = dy * mm_per_px
            # print(f"dx_mm:{self.dx_mm}  dy_mm:{self.dy_mm}")

            dx_mm_temp = -(self.dy_mm-90)/1000
            dy_mm_temp = -(self.dx_mm+15)/1000
            self.dx_mm_trans = dx_mm_temp
            self.dy_mm_trans = dy_mm_temp

            # print(f"dx_mm_trans:{self.dx_mm}  dy_mm_trans:{self.dy_mm}")

            # print(f"current_pos:{self.current_pos}")
            self.pos_extected = [self.current_pos[0]+self.dx_mm_trans, self.current_pos[1]+self.dy_mm_trans, 0.7]
            # print(f"pos_extected:{self.pos_extected}")


            # 横/竖红线
            cv2.line(output, (cx_img, cy_img), (xc, cy_img), (0, 0, 255), 2)
            cv2.line(output, (xc, cy_img), (xc, yc), (0, 0, 255), 2)
            # 圆环和圆心
            for x, y, r in group:
                cv2.circle(output, (x0 + x, y0 + y), r, (0, 255, 0), 2)
            cv2.circle(output, (xc, yc), 3, (0, 0, 255), -1)
            # 右下角显示dx_mm和dy_mm
            margin = 10
            txt1 = f"dx: {self.dx_mm:.1f} mm"
            txt2 = f"dy: {self.dy_mm:.1f} mm"
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.8
            thickness = 2
            ((tw1, th1), _) = cv2.getTextSize(txt1, font, scale, thickness)
            ((tw2, th2), _) = cv2.getTextSize(txt2, font, scale, thickness)
            bx = w - max(tw1, tw2) - margin
            by1 = h - margin - th2 - 5
            by2 = h - margin
            cv2.putText(output, txt1, (bx, by1), font, scale, (0,0,255), thickness)
            cv2.putText(output, txt2, (bx, by2), font, scale, (0,0,255), thickness)
            if match_template:
                cv2.putText(output, "Triple rings match template!", (20, 40),
                            font, 1, (0, 200, 255), 2)
            else:
                cv2.putText(output, "Triple concentric rings detected", (20, 40),
                            font, 1, (0, 255, 0), 2)
        else:
            cv2.putText(output, "No triple rings detected", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return output

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
            if color_name == "orange":
                mask1 = cv2.inRange(hsv_img, thresholds["lower"], thresholds["upper"])
                mask2 = cv2.inRange(hsv_img, thresholds["lower2"], thresholds["upper2"])
                mask = cv2.bitwise_or(mask1, mask2)
                # mask = mask2
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
                cv2.putText(image, color_name.upper(), (center_x - 20, center_y - 20),
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
        
        image = self.process_frame(image, self.template_circles)
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
    # while rclpy.ok() :
    #     rclpy.spin_once(node)
    #     node.get_logger().info(f"111")
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
