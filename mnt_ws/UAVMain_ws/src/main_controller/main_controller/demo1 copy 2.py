# 1. flight mode change OFFBOARD
# 2. clip test
# 3. fly altitude 1 meter and hold on
# 4. give a point 
# 5. fly back and land

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from ego_planner.srv import SetPosition
from main_controller.position_checker_ros2 import wait_until_position_reached,is_position_reached
from rcl_interfaces.srv import GetParameters, SetParameters
import time
from std_srvs.srv import SetBool  # 使用标准布尔服务类型
from std_msgs.msg import Int32  # 导入 Int32 消息类型
import logging
from geometry_msgs.msg import Point
import math
from datetime import datetime
import os
from nav_msgs.msg import Odometry
import sys  # 用于删除目录及其内容
import numpy as np
import re


def transform_coordinates_dict(coord_dict):
    """
    坐标转换: (x, y, z) -> (y, -x, z)
    参数:
        coord_dict: dict[int, list[float]]  # {id: [x, y, z]}
    返回:
        转换后的字典
    """
    return {k: [v[1], -v[0], v[2]] for k, v in coord_dict.items()}



class UAVController(Node):

    # destination = {1 : [3.0, 2.0, 0.8], 2 : [3.0, 0.0, 0.8] , 3 : [3.0, -2.0, 0.8]}  # 假设有多个目标位置
    destination_ori = {1 : [-1.52, 5.13, 0.7], 2 : [-0.03, 5.12, 0.7] , 3 : [1.47, 5.12, 0.7]}  # 假设有多个颜色目标位置
    put_destination_ori = {1 : [-1.50, 6.31, 0.7], 2 : [-0.01, 6.30, 0.7] , 3 : [1.53, 6.32, 0.7]}  # 投放位置
    # destination = {1 : [0.5, -0.5, 0.3], 2 : [0.5, 0.0, 0.3] , 3 : [0.5, 0.5, 0.3]}  # 假设有多个目标位置
    color = None

    destination = transform_coordinates_dict(destination_ori)
    put_destination = transform_coordinates_dict(put_destination_ori)


    def __init__(self):
        super().__init__('uav_controller')

        self.setpoint_received = False  # 检测 setpoint 是否非空
        self.init_position = None  # 用于存储接收到的初始位置
        self.current_state = None  # 保存当前状态
        # 计算出的预期投放位置
        self.put_des_exp = None
        # 当前位置
        self.current_pos = None

        # 订阅 /mavros/setpoint_raw/local 话题
        self.setpoint_raw_subscriber = self.create_subscription(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            self.setpoint_raw_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        )

        # 订阅 /mavros/state 话题
        self.state_subscriber = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        )
        # 订阅 /init_position 话题
        self.init_position_subscriber = self.create_subscription(
            Point,
            'init_position',
            self.init_position_callback,
            10
        )
        # 订阅当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            "Odometry",
            self.odom_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        # 创建服务客户端，用于切换飞行模式
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client_deepseek = self.create_client(GetParameters, '/deepseek_node/get_parameters')
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 创建发布者
        self.color_check_client = self.create_client(SetBool, '/check_color_match')
        self.get_distance_error_values_client = self.create_client(SetBool, '/get_distance_error_values')
        self.set_position_client = self.create_client(SetPosition, '/set_position')
        
        

    def setpoint_raw_callback(self, msg):
        # 检测 /mavros/setpoint_raw/local 中的成员是否非空
        if msg.position.x is not None and msg.position.y is not None and msg.position.z is not None:
            self.setpoint_received = True
        else:
            self.get_logger().warning("Received setpoint with empty fields!")

    def state_callback(self, msg):
        # 保存当前无人机状态
        self.current_state = msg
    # 订阅回调函数，用于接收 init_position 话题的数据
    def init_position_callback(self, msg):
        """订阅回调函数，用于接收 init_position 话题的数据"""
        self.init_position = [msg.x, msg.y, msg.z]

    def odom_callback(self, msg):
        # 更新位置
        pos = msg.pose.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])
            
    # 设置飞行模式为 OFFBOARD
    def set_offboard_mode(self):
        """设置飞行模式为 OFFBOARD"""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mavros/set_mode 服务...')

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'OFFBOARD'

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().mode_sent:
            self.get_logger().info('成功设置飞行模式为 OFFBOARD')
        else:
            self.get_logger().error('设置飞行模式失败')

    #获取目标颜色
    def get_remote_param_deepseek(self,param):
        req = GetParameters.Request()
        req.names = [param]  # 请求参数名
        # future = self.param_client_deepseek.call(req)
        future = self.param_client_deepseek.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        # 检查服务调用结果
        if future.result() is None:
            self.get_logger().warning(f"Service call to get parameter {param} failed or timed out")
            return None

        # 检查返回值是否包含参数
        if not future.result().values:
            self.get_logger().warning(f"Parameter {param} not found or empty")
            return None
        return future.result().values[0].integer_value  # 假设返回的是一个列表，取第一个值

    # 操控夹爪
    def publish_pwm_duty_cycle(self, duty_cycle):
        """发布 PWM 占空比到 /pwm/duty_cycle 话题"""
        pwm_publisher = self.create_publisher(Int32, '/pwm/duty_cycle', 10)
        msg = Int32()
        msg.data = duty_cycle
        pwm_publisher.publish(msg)
        self.get_logger().info(f"Published PWM duty cycle: {duty_cycle}")

    def check_offboard_mode(self):
        """检测是否成功切换到 OFFBOARD 模式"""
        
        while self.current_state == None:
            self.get_logger().info(f'current_state = {self.current_state}')
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info(f'current_state = {self.current_state}')
        
        if self.current_state.mode == 'OFFBOARD':
            self.get_logger().info('无人机已成功切换到 OFFBOARD 模式')
            return True
        else:
            self.get_logger().warning('无人机未切换到 OFFBOARD 模式')
            return False

    # 发位置
    def publish_goal(self, position,position_th=None):
        """根据颜色键值发布目标位置到 /move_base_simple/goal"""

        goal = PoseStamped()
        # goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "world"
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        goal.pose.orientation.x = 0.0  # 单位四元数
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0  # 单位四元数
        self.goal_publisher.publish(goal)
        self.get_logger().info(f'==========================已发布目标位置: x={position[0]}, y={position[1]}, z={position[2]}=======================================================================')

        is_reach = False
        # 等待直到 UAV 到达目标位置
        if position_th is None:
            is_reach = wait_until_position_reached(self, target_position=position)
        else:
            is_reach = wait_until_position_reached(self, target_position=position,position_threshold=position_th)
        
        if not is_reach:
            self.get_logger().error('UAV 未能到达目标位置，任务终止 ' + position)
            return
    
    # 调服务获取预期位置
    def get_distance_error_values(self):
        """获取距离误差值"""
        while not self.get_distance_error_values_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /get_distance_error_values 服务...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        req = SetBool.Request()
        req.data = True  # 实际未使用，但需要填充
        future = self.get_distance_error_values_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
    
        if future.result() is not None:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'获取距离误差值失败: {response.message}')
                return None
            
            # 提取 x 和 y 的值
            try:
                message = response.message
                # 使用正则表达式提取 pos_extected 的值
                match = re.search(r"pos_extected: \[(.*?)\]", response.message)
                if match:
                    # 提取括号内的内容并将其转换为浮点数数组
                    pos_extected_array = [float(x) for x in match.group(1).split(",")]
                    print("Extracted pos_extected array:", pos_extected_array)
                return pos_extected_array
            except Exception as e:
                self.get_logger().error(f"解析 response.message 失败: {e}")
                return None
        else:
            self.get_logger().error('服务调用失败')
            return None  
        
    def call_set_position_service(self, x, y,z):
        """
        调用 /set_position 服务
        参数：
            node: 当前 ROS2 节点
            x: 目标位置的 x 坐标
            y: 目标位置的 y 坐标
        返回：
            (bool, str): 服务调用是否成功，以及返回的消息
        """

        # 等待服务可用      
        if not self.set_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待 /set_position 服务...')
            rclpy.spin_once(self, timeout_sec=1.0)

        # 创建请求
        request = SetPosition.Request()
        request.x = x
        request.y = y
        request.z = z  # 添加 z 坐标

        # 异步调用服务
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # 处理响应
        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        else:
            return False, "服务调用失败，无响应"

    # 获取初始位置数据
    def get_init_position(self):
        """获取初始位置数据"""
        while self.init_position is None:
            self.get_logger().info("等待接收 init_position 数据...")
            rclpy.spin_once(self, timeout_sec=1.0)
        return self.init_position

def module0(node):
    """模块 0：切换飞行模式为 OFFBOARD"""
    while not node.setpoint_received:
        logging.getLogger().warning("Module 0: setpoint_raw_callback 未接收到有效的 setpoint")
        rclpy.spin_once(node, timeout_sec=1.0)
    
    node.set_offboard_mode()

    # 检测是否成功切换到 OFFBOARD 模式
    if not node.check_offboard_mode():
        logging.getLogger().warning("Module 0: OFFBOARD 模式切换失败")
       
    logging.getLogger().info("Module 0: OFFBOARD 模式切换成功")
    return True

# 检测起飞后点位
def module2(node):
    success = wait_until_position_reached(node,
        target_position=[0, 0, 0.7],position_threshold=0.1,timeout_sec=400.0,direction="z")  
    return success

# 获取颜色
def module1(node):
    # 循环检测是否从 deepseek 处获取颜色
    while rclpy.ok():
        node.color = node.get_remote_param_deepseek('aim_color')
        if node.color is not None and node.color != 0:
            break
        rclpy.spin_once(node, timeout_sec=1.0)
    logging.getLogger().info(f"color = {node.color}")

# 发送点位并且识别
def module3(node:UAVController):

    # 发布目标位置到 /move_base_simple/goal
    time.sleep(3)
    print(f"Module 3: 发布目标位置: {node.destination[node.color]}")
    node.publish_goal(node.destination[node.color],position_th=0.15)
    

    # use color_detect  td
    req = SetBool.Request()
    req.data = True  # 实际未使用，但需要填充
    
    node.future = node.color_check_client.call_async(req)
    rclpy.spin_until_future_complete(node, node.future, timeout_sec=5.0)
    
    
    if node.future.result() is not None:
        response = node.future.result()
        node.get_logger().info(f'结果: {response.success}, 消息: {response.message}')
        # color detect success
    else:
        node.get_logger().error('服务调用失败')




    # # tong ji ci shu
    # cnt = 0
    # while rclpy.ok() and cnt < 3:
    #     # 发布目标位置到 /move_base_simple/goal
    #     node.publish_goal(node.destination[node.color],position_th=0.15)

    #     # use color_detect  td
    #     req = SetBool.Request()
    #     req.data = True  # 实际未使用，但需要填充
        
    #     node.future = node.color_check_client.call_async(req)
    #     rclpy.spin_until_future_complete(node, node.future, timeout_sec=5.0)
        
        
    #     if node.future.result() is not None:
    #         response = node.future.result()
    #         node.get_logger().info(f'结果: {response.success}, 消息: {response.message}')
    #         # color detect success
    #         if response.success:
    #             break
    #     else:
    #         node.get_logger().error('服务调用失败')

    #     node.color = (node.color + 1) % 4
    #     if node.color == 0:
    #         node.color = 1
    #     cnt += 1
    #     rclpy.spin_once(node, timeout_sec=1.0)

# 颜色检测测试
def module4(node):
    # use color_detect  td
    req = SetBool.Request()
    req.data = True  # 实际未使用，但需要填充
    
    future = node.color_check_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    # 检查服务调用是否成功
    if future.done():
        try:
            result = future.result()
            logging.getLogger().info(f"Service call succeeded: {result}")
            return result.success
        except Exception as e:
            logging.getLogger().error(f"Service call failed: {e}")
            return None
    else:
        logging.getLogger().warning("Service call did not complete in time")
        return None

# 飞到投放位置
def module5(node):
    """模块 5：飞到投放位置"""
    # 发布目标位置到 /move_base_simple/goal
    # pos = [node.put_destination[node.color][0] + 0.55,node.destination[node.color][1],0.4]
    pos = node.put_destination[node.color]
    init_position = node.get_init_position()
    # 调用 /set_position 服务
    success, message = node.call_set_position_service(pos[0]+init_position[0]-0.23, pos[1]+init_position[1],pos[2]+init_position[2])
    node.get_logger().info(f"Module 5: 调用 /set_position 服务，目标位置: {pos[0]+init_position[0]-0.23}, {pos[1]+init_position[1]}, {pos[2]+init_position[2]}")

    if success:
        node.get_logger().info(f"Module 5: 成功调用 /set_position 服务，消息: {message}")
    else:
        node.get_logger().error(f"Module 5: 调用 /set_position 服务失败，消息: {message}")
    
    # 等待 UAV 到达目标位置
    if not wait_until_position_reached(node, target_position=[pos[0]+init_position[0]-0.23, pos[1]+init_position[1],pos[2]+init_position[2]],
                                        position_threshold=0.1,timeout_sec=6):
        node.get_logger().error('UAV 未能到达投放位置，任务终止')
    time.sleep(2)
    
    node.get_logger().info("Module 5: UAV 已到达投放位置")

    # 获取误差计算预期值,降落时要用
    cal_expected_pos(node)


    return True

# 操作夹爪 
def module7(node:UAVController):


    """获取初始位置数据"""
    while node.current_pos is None:
        node.get_logger().info("等待接收 当前位置 数据...")
        rclpy.spin_once(node, timeout_sec=1.0)


    """模块 5：发布 PWM 占空比"""
    node.get_logger().warn(f"========== 投放位置为{node.current_pos} =============")
    duty_cycle = 91  # 设置占空比
    node.publish_pwm_duty_cycle(duty_cycle)

# 获取距离投放位置的误差值，并降低位置
def module6(node):
    # node.get_logger().info("Module 6: 获取距离投放位置的误差值")

    # x_value, y_value = None, None
    # # x_value, y_value = 0.2, 0.2
    # # td 要加 z 方向的值
    # while x_value is None or y_value is None:
    #     x_value, y_value = node.get_distance_error_values()
    #     node.get_logger().info(f"Module 6: 当前误差值 x={x_value}, y={y_value}")

    # 调用 /set_position 服务

    pos = node.put_des_exp
    success, message = node.call_set_position_service(pos[0],pos[1],0.3)
    if success:
        node.get_logger().info(f"Module 6: 成功调用 /set_position 服务,发送位置为{node.put_des_exp}，消息: {message}")
    else:
        node.get_logger().error(f"Module 6: 调用 /set_position 服务失败,发送位置为{node.put_des_exp}，消息: {message}")
    
    wait_until_position_reached(node, target_position=[pos[0],pos[1],0.3], position_threshold=0.1, timeout_sec=5)

        
# 飞回起飞的位置
def module8(node):
    node.get_logger().info("Module 8: 返回起飞位置")
    # 获取初始位置
    init_position = node.get_init_position()
    node.get_logger().info(f"Module 8: 初始位置 x={init_position[0]}, y={init_position[1]}, z=0.6")
    node.publish_goal([init_position[0],init_position[1],0.6])

def calculate_distance(point1, point2):
    """
    计算两个点之间的欧几里得距离
    :param point1: 第一个点，格式 [x1, y1, z1]（3D）或 [x1, y1]（2D）
    :param point2: 第二个点，格式 [x2, y2, z2]（3D）或 [x2, y2]（2D）
    :return: 两点之间的距离（float）
    """
    if len(point1) != len(point2):
        raise ValueError("坐标维度不一致！")
    
    squared_distance = sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2))
    return math.sqrt(squared_distance)

# 微调投放的位置，获取误差计算预期值
def cal_expected_pos(node:UAVController):

    node.put_des_exp = node.get_distance_error_values()
    print(f"================= 预期位置计算成功为：{node.put_des_exp} =====================")

    if node.put_des_exp  == None:
        node.get_logger().warn("===================没有获取到误差，再试一次===============")
        time.sleep(0.5)
        node.put_des_exp  = node.get_distance_error_values()
        if node.put_des_exp == None:
            node.put_des_exp = node.put_destination[node.color]
            return

    if calculate_distance(node.put_des_exp,node.put_destination[node.color]) > 0.3:
        node.put_des_exp = node.put_destination[node.color]
        return
    node.put_des_exp = [node.put_des_exp[0] ,node.put_des_exp[1],0.3]
    node.get_logger().info(f"================= 预期位置计算成功为：{node.put_des_exp} =====================")
    


def main(args=None):
    rclpy.init(args=args)

    # 设置日志记录
    # setup_logging()
    # logger = logging.getLogger(__name__)
    # logger.info("Program started")
    
    input_value = 0  # 默认值为 0
    if len(sys.argv) > 1:  # 检查是否有额外参数
        try:
            # 获取传入的整型参数
            input_value = int(sys.argv[1])
        except ValueError:
            print("Invalid input value. Using default value 1.")


    # 掩码值，决定运行哪个模块
    mask = 0  # 这里设置为运行第一个模块
    node = UAVController()
    # color = None

    while rclpy.ok():
        try:
            # 切换为offboard模式
            if mask == 0:  # 检查掩码值是否启用第一个模块
                result = module0(node)
                if result:
                    node.get_logger().info("Module 0 executed successfully")
                    node.get_logger().info(f'{mask}')
                    mask += 1
            # 获取颜色
            # logging.getLogger().info(f"mask{mask} started")
            if mask == 1:
                node.get_logger().info("Module 1 executed")
                # module1(node)
                node.color = input_value if len(sys.argv) > 1 else 1  # 使用传入的参数或默认值
                node.get_logger().info("Module 1 executed successfully")
                mask += 1
                # mask = 3
            # 检测是否起飞
            if mask == 2:
                module2(node)
                node.get_logger().info("Module 2 executed successfully")
                time.sleep(2)
                mask += 1
            # 去查询颜色
            if mask == 3:
                # node.publish_goal([3.0,0.0,0.5])
                module3(node)
                time.sleep(5)
                node.get_logger().info("Module 3 executed successfully")
                mask = 5
            # 颜色检测
            if mask == 4:
                result = module4(node)
                node.get_logger().info(f"result: {result}")
                if result:
                    mask += 1
                else:
                    node.get_logger().warning(f"Module 4 failed, retrying... mask={mask}")
                node.get_logger().info("Module 4 executed successfully")
                break
            # 飞到投放位置
            if mask == 5:
                module5(node)
                time.sleep(2)
                node.get_logger().info("Module 5 executed successfully")
                mask += 1

            # 控制器进行微调，并且降低
            if mask == 6:
                module6(node)
                time.sleep(3)
                mask += 1

            # 夹爪打开
            if mask == 7:
                module7(node)
                time.sleep(1)
                node.get_logger().info("Module 6 executed successfully")
                mask += 1

            # 返回降落点
            if mask == 8:
                module8(node)
                rclpy.spin(node)
                break
               
            rclpy.spin_once(node, timeout_sec=1.0)  
        except Exception as e:
            node.get_logger().error(f"An error occurred: {e}")
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
