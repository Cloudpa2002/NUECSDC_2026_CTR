# 睿抗

import sys
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ego_planner.srv import SetPlannerMode
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from tools.position_checker_ros2 import wait_until_position_reached
import time
import subprocess
import numpy as np

class Initialization(Node):
    """
    此类负责初始化工作
    包含以下子功能:
        1.创建话题订阅方、发布方
        2.创建服务客户端
        3.通过回调函数实时更新有关数据
    """
    def __init__(self):
        """
        功能: 初始化 Initialization 节点，创建话题订阅方与发布方，创建服务客户端
        输入: 无
        输出: 无
        """
        super().__init__('uav_controller_test')

        self.setpoint_received = False
        self.current_pos = None

        # ==================== 订阅方 ====================
        # 订阅期望位置
        self.setpoint_raw_subscriber = self.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.setpoint_raw_callback, 
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        # 订阅当前位置
        self.odom_sub = self.create_subscription(
            Odometry, "Odometry", self.odom_callback, 
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        
        # ==================== 发布方 ====================
        # 发布目标位置
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # ==================== 服务端 ====================
        # 切换 mavros 模式
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.planner_mode_client = self.create_client(SetPlannerMode, '/set_planner_mode')

    # ==================== 回调函数 ====================
    def setpoint_raw_callback(self, msg):
        """
        功能: 检测 /mavros/setpoint_raw/local 话题中的位置字段是否非空，非空则置位 setpoint_received 标志
        输入: msg - PositionTarget 消息，包含 position、velocity 等字段
        输出: 无直接输出，更新 self.setpoint_received 标志
        """
        if msg.position.x is not None and msg.position.y is not None and msg.position.z is not None:
            self.setpoint_received = True

    def odom_callback(self, msg):
        """
        功能: 接收里程计数据，更新无人机当前位置
        输入: msg - nav_msgs/Odometry 消息，包含位姿和速度信息
        输出: 无直接输出，更新 self.current_pos 为 numpy 数组 [x, y, z]
        """
        pos = msg.pose.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])

    def set_planner_mode(self, mode, timeout_sec=2.0):
        """
        功能: 调用 /set_planner_mode 服务，切换 EGO Planner 运行时参数模式
        输入: mode - ONE/TWO/THREE/FOUR/NORMAL
        输出: bool - True 表示切换成功，False 表示切换失败
        """
        if not self.planner_mode_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('/set_planner_mode 服务不可用')
            return False

        request = SetPlannerMode.Request()
        request.mode = mode

        future = self.planner_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f'切换 planner mode 超时: {mode}')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(f'切换 planner mode 失败，无响应: {mode}')
            return False

        if response.success:
            self.get_logger().info(f'planner mode 已切换为 {mode}: {response.message}')
        else:
            self.get_logger().error(f'planner mode 被拒绝: {response.message}')

        return response.success

class UAV_Status:
    """
    此类用于存储无人机基础状态控制相关的方法。如电门、mavros状态等。
    包含以下子功能:
        1.解锁电门
        2.切换mavros状态
    """
    def __init__(self, node):
        """
        功能: 绑定 Initialization 节点实例，以便访问 set_mode_client、日志等资源
        输入: node - Initialization 节点实例
        输出: 无
        """
        self.node = node

    def run_arm_command_in_home(self):
        """
        功能: 在主目录终端环境下执行 arm 命令，用于解锁无人机电门
        输入: 无
        输出: True
        """
        command = "cd ~ && arm"
        self.node.get_logger().info(f'准备在主目录终端环境执行命令: {command}')

        subprocess.run(
            ["bash", "-ic", command],
            cwd=os.path.expanduser("~"),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(1.0)
        return True

    def set_offboard_mode(self):
        """
        功能: 调用 /mavros/set_mode 服务，将无人机飞行模式切换为 OFFBOARD
        输入: 无
        输出: 无直接返回值，通过日志报告切换结果
        """
        self.node.get_logger().info('准备切换至 OFFBOARD 模式')

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'OFFBOARD'

        future = self.node.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() and future.result().mode_sent:
            self.node.get_logger().info('成功进入 OFFBOARD 模式')
            return True
        else:
            self.node.get_logger().error('进入 OFFBOARD 失败！')
            return False

    def land(self):
        """
        功能: 调用 /mavros/set_mode 服务，将无人机切换至 AUTO.LAND 模式执行安全降落
        输入: 无
        输出: bool - True 表示成功触发降落模式，False 表示触发失败
        """
        self.node.get_logger().info('准备切换至 AUTO.LAND 模式')

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'AUTO.LAND'

        future = self.node.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() and future.result().mode_sent:
            self.node.get_logger().info('成功进入 AUTO.LAND')
            return True
        else:
            self.node.get_logger().error('进入 AUTO.LAND 失败！')
            return False

class Position_Control:
    """
    此类包含较为基础的无人机任务控制方法。
    包含以下子功能:
        1.发布目标点
        2.阻塞式等待无人机到达目标点
    """
    def __init__(self, node):
        """
        功能: 绑定 Initialization 节点实例，初始化位姿相关数据
        输入: node - Initialization 节点实例
        输出: 无
        """
        self.node = node
        self.current_pos = None

    def publish_goal(self, position, position_th=None, velocity_th=None):
        """
        功能: 向 /move_base_simple/goal 话题发布目标位置，并通过 wait_until_position_reached 阻塞等待无人机到达
        输入: position - list [x, y, z]，目标位置坐标
              position_th - float 或 None，位置到达阈值，None 时使用默认阈值
        输出: bool - True 表示成功到达目标位置，False 表示未能在规定时间内到达
        """
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        goal.pose.orientation.w = 1.0
        self.node.goal_publisher.publish(goal)
        self.node.get_logger().info(f'已发布目标位置: x={position[0]}, y={position[1]}, z={position[2]}')

        # 等待到达目标点
        if position_th is None:
            is_reach = wait_until_position_reached(self.node, target_position=position)
        else:
            is_reach = wait_until_position_reached(self.node, target_position=position, position_threshold=position_th, velocity_threshold=velocity_th)

        if not is_reach:
            self.node.get_logger().error('未能在规定时间内到达目标位置')
            return False
        return True

class Mask:
    """
    此类包含 move_test 的具体流程定义
    """
    def __init__(self, node, target_pos_preland=None, target_pos_1=None, target_pos_hover=None):
        """
        功能: 初始化
        输入: node - Initialization 节点实例
             target_pos_preland - list [x, y, z]，预降落点
             target_pos_1 - list [x, y, z]，第一个目标点
             target_pos_hover - list [x, y, z]，起飞悬停点
        输出: 无
        """
        self.node = node
        self.target_pos_preland = target_pos_preland
        self.target_pos_1 = target_pos_1
        self.target_pos_hover = target_pos_hover
        self.pos_control = Position_Control(node)
        self.status = UAV_Status(node)

    def mask0(self):
        """
        功能: 等待 setpoint 就绪，再执行 arm 解锁，最后切换为 OFFBOARD 模式
        输入: 无
        输出: bool - True 表示 OFFBOARD 模式切换成功，False 表示失败
        """
        # 1. 等待 setpoint 就绪
        while not self.node.setpoint_received and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # 2. 主目录终端arm & 切换 OFFBOARD 模式
        if self.status.run_arm_command_in_home() and self.status.set_offboard_mode():
            return True
        else:
            return False

    def mask1(self):
        """
        功能: 起飞至指定高度 (z=0.6)，仅检测 z 方向
        输入: 无
        输出: bool - True 表示已到达起飞高度，False 表示超时或失败
        """
        self.node.get_logger().info("正在起飞...")
        success = wait_until_position_reached(
            self.node, target_position=self.target_pos_hover,
            position_threshold=0.4, timeout_sec=30.0)
        if success:
            time.sleep(0.5)
            return True
        else:
            self.node.get_logger().error("起飞超时或失败")
            return False

    def mask2(self):
        """
        功能: 前往预降落点，然后下降到低高度等待 AUTO.LAND
        输入: 无
        输出: bool - 全部到达返回 True，任一失败返回 False
        """

        pos_land = self.target_pos_preland.copy()
        pos_land[2] = 0.1

        if not self.node.set_planner_mode('THREE'):
            return False
        self.node.get_logger().info(f"前往预降落点: {self.target_pos_1}")
        if not self.pos_control.publish_goal(self.target_pos_1, position_th=0.2):
            self.node.get_logger().error("前往预降落点失败")
            return False
        # 前往预降落点
        if not self.node.set_planner_mode('THREE'):
            return False
        self.node.get_logger().info(f"前往预降落点: {self.target_pos_preland}")
        if not self.pos_control.publish_goal(self.target_pos_preland, position_th=0.2):
            self.node.get_logger().error("前往预降落点失败")
            return False
        
        self.node.get_logger().info(f"正在下降高度")
        if not self.node.set_planner_mode('FOUR'):
            return False
        if not self.pos_control.publish_goal(pos_land, position_th=0.2):
            self.node.get_logger().error("下降高度失败")
            return False

        return True

    def mask3(self):
        return True

    def mask4(self):
        """
        功能: 执行 AUTO.LAND 安全降落，完成后 spin 释放资源
        输入: 无
        输出: bool - True 表示成功触发降落并完成退出，False 表示降落触发失败
        """
        # 执行降落
        self.node.get_logger().info("准备执行安全降落程序...")
        if not self.status.land():
            self.node.get_logger().error("降落触发失败")
            return False

        # 降落完成后 spin 释放资源
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return True

def main(args=None):
    """
    功能: 程序主入口，初始化 ROS2 节点并按 mask 顺序执行各模块
    输入: args - 命令行参数，传递给 rclpy.init()
    输出: 无
    """
    rclpy.init(args=args)
    node = Initialization()

    target_pos_hover = [0.0, 0.0, 0.6]     # 起飞悬停点
    target_pos_1= [2.3, 0.0, 0.6]
    target_pos_preland = [4.1, 0.0, 0.6]    # 预降落点

    mask = Mask(node, target_pos_preland=target_pos_preland, target_pos_1=target_pos_1, target_pos_hover=target_pos_hover)
    current = 0

    while rclpy.ok():
        try:
            if current == 0:
                if mask.mask0():
                    node.get_logger().error("Mask 0 完成")
                    current += 1
                else:
                    node.get_logger().error("Mask 0 失败，终止任务")
                    break

            elif current == 1:
                if mask.mask1():
                    current += 1
                    node.get_logger().error("Mask 1 完成")
                else:
                    node.get_logger().error("Mask 1 失败，终止任务")
                    break

            elif current == 2:
                if mask.mask2():
                    current += 1
                    node.get_logger().error("Mask 2 完成")
                else:
                    node.get_logger().error("Mask 2 失败，终止任务")
                    break

            elif current == 3:
                if mask.mask3():
                    current += 1
                    node.get_logger().error("Mask 3 完成")
                else:
                    node.get_logger().error("Mask 3 失败，终止任务")
                    break

            elif current == 4:
                if mask.mask4():
                    node.get_logger().error("Mask 4 完成")
                    break
                else:
                    node.get_logger().error("Mask 4 失败，终止任务")
                    break

            rclpy.spin_once(node, timeout_sec=0.05)

        except Exception as e:
            node.get_logger().error(f"发生异常: {e}")
            break
        except KeyboardInterrupt:
            node.get_logger().info("用户手动中断")
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

