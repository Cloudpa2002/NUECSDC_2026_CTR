#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from position_checker_ros2 import wait_until_position_reached
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from mavros_msgs.srv import SetMode  # 导入服务类型
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool  # 使用标准布尔服务类型

class UAVController(Node):
    def __init__(self):
        super().__init__('UAVController')
        # 等待其他节点启动
        time.sleep(5)
        self.param_client_deepseek = self.create_client(GetParameters, '/deepseek_llm_ros/get_parameters')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')  # 创建服务客户端
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)  # 创建发布者
        self.color_check_client = self.create_client(SetBool, '/check_color_match')
        # td
        self.param_client_gripper = self.create_client(SetParameters, '/deepseek_node/get_parameters')


    destination = {1 : [3, -2, 1], 2 : [3, 0, 1] , 3 : [3, 2, 1]}  # 假设有多个目标位置

    """设置飞行模式为 OFFBOARD"""
    def set_offboard_mode(self):
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mavros/set_mode 服务...')
        
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'OFFBOARD'

        # future = self.set_mode_client.call(request)
        future = self.param_client_deepseek.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.mode_sent:
            self.get_logger().info('成功设置飞行模式为 OFFBOARD')
        else:
            self.get_logger().error('设置飞行模式失败')

    def set_gripper(self,is_pinch):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mavros/set_mode 服务...')


    # 获取目标颜色
    def get_remote_param_deepseek(self,param):
        req = GetParameters.Request()
        req.names = [param]  # 请求参数名
        # future = self.param_client_deepseek.call(req)
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
    

    def publish_goal(self, position):
        """根据颜色键值发布目标位置到 /move_base_simple/goal"""

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "world"
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        goal.pose.orientation.x = 0.0  # 单位四元数
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0  # 单位四元数
        self.goal_publisher.publish(goal)
        self.get_logger().info(f'已发布目标位置: x={position[0]}, y={position[1]}, z={position[2]}')

        # 等待直到 UAV 到达目标位置
        is_reach = wait_until_position_reached(self, target_position=position)
        
        if not is_reach:
            self.get_logger().error('UAV 未能到达目标位置，任务终止 ' + position)
            return


    def destroy_node(self):
        """清理资源"""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UAVController()

    # 循环检测是否从 deepseek 处获取颜色
    color = None
    while rclpy.ok():
        color = node.get_remote_param_deepseek('aim_color')
        if color is not None and color != 0:
            break

    # 调用设置 OFFBOARD 模式
    node.set_offboard_mode()

    # 等待直到 UAV 到达目标位置
    is_reach = wait_until_position_reached(node, target_position=[0, 0, 1],
        position_threshold=0.10,
        velocity_threshold=0.05,
        timeout_sec=300)
    
    if not is_reach:
        node.get_logger().error('UAV 未能到达目标位置，任务终止 ' + '0,0,1')
        return

    node.get_logger().info('UAV takeoff success')


    # tong ji ci shu
    cnt = 0
    while rclpy.ok() and cnt < 3:
        # 发布目标位置到 /move_base_simple/goal
        node.publish_goal(node.destination[color])

        # use color_detect  td
        req = SetBool.Request()
        req.data = True  # 实际未使用，但需要填充
        
        node.future = node.color_check_client.call_async(req)
        rclpy.spin_until_future_complete(node, node.future, timeout_sec=5.0)
        
        if node.future.result() is not None:
            response = node.future.result()
            node.get_logger().info(f'结果: {response.success}, 消息: {response.message}')
            # color detect success
            if response.success:
                break
        else:
            node.get_logger().error('服务调用失败')

        color = (color + 1) % 4
        if color == 0:
            color = 1
        cnt += 1


    # fei dao mu biao shang fang 
    # 发布目标位置到 /move_base_simple/goal

    pos = [node.destination[color][0] + 0.5,node.destination[color][1],0.5]
    node.publish_goal(pos)

    # td set gripper
    node.set_gripper(True)

    time.sleep(3)

    node.publish_goal([0,0,1])

    # land




    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()