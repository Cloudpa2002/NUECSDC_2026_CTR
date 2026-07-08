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
from main_controller.position_checker_ros2 import wait_until_position_reached
from main_controller.UAV_utils import control_gpio
from rcl_interfaces.srv import GetParameters, SetParameters
import time
from std_srvs.srv import SetBool  # 使用标准布尔服务类型
from std_msgs.msg import Int32,String  # 导入 Int32 消息类型
import logging
from geometry_msgs.msg import Point



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

    # 0代表向前一点防止撞网,1代表字母识别位置，2代表投放位置，3代表回来投放位置附近，4代表字母附近
    destination_ori = { 0 : [0.32,1.19,0.65],1 : [0.03, 3.50, 0.65], 2 : [-3.05, 3.31, 0.65],3 : [-1.70,2.17,0.4],4 : [0.03,3.26,0.4]}  # 假设有多个目标位置
    # td modify
    # hit_destination_ori = {0 : [-1.65, 0.5, 0.4], 1 : [-2.22, 0.5, 0.4],2:[-2.81, 0.5, 0.4]}  # 假设有多个目标位置
    # hit_destination_ori = {0 : [-1.65, 0.5, 0.65], 1 : [-2.22, 0.5, 0.65],2:[-2.81, 0.5, 0.65]}  # 假设有多个目标位置

    # 击打位置高度
    hit_pos_altitude = 1.00
    # 击打位置第二个坐标
    hit_pos_y = 0.56
    hit_destination_ori = {0 : [-1.70, hit_pos_y, 0.65], 1 : [-2.27, hit_pos_y, 0.65],2:[-2.85, hit_pos_y, 0.65]}  # 假设有多个目标位置
    destination = transform_coordinates_dict(destination_ori)
    hit_destination =transform_coordinates_dict(hit_destination_ori)



    aim_letter = 1  # 默认值为 0
    letter_count = [0,0,0] # 假设有三个字母
    flag = 0  # 用于标记是否已经飞到字母位置
    def __init__(self):
        super().__init__('uav_controller')

        self.setpoint_received = False  # 检测 setpoint 是否非空
        self.init_position = None  # 用于存储接收到的初始位置
        self.current_state = None  # 保存当前状态

        # 先关激光灯
        control_gpio(103, value=0)  # 关闭激光灯


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
            '/init_position',
            self.init_position_callback,
            10
        )

        self.yolov5_results_subscriber = self.create_subscription(
            String,
            '/yolov5_results',
            self.yolov5_results_callback,
            10
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

     # 创建订阅器，订阅 "yolov5_results" 话题
    
    def yolov5_results_callback(self,msg):
        """处理 yolov5_results 话题的回调函数"""
        self.get_logger().info(f"接收到的目标检测结果: {msg.data}")
        if msg.data == "A":
            self.letter_count[0] += 1
        elif msg.data == "B":
            self.letter_count[1] += 1
        elif msg.data == "C":
            self.letter_count[2] += 1
        

    def state_callback(self, msg):
        # 保存当前无人机状态
        self.current_state = msg
    # 订阅回调函数，用于接收 init_position 话题的数据
    def init_position_callback(self, msg):
        """订阅回调函数，用于接收 init_position 话题的数据"""
        self.init_position = [msg.x, msg.y, msg.z]
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
    
    # 调服务获取误差  td 删除不删除
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
                return None, None
            
            # 提取 x 和 y 的值
            try:
                message = response.message
                x_value = float(message.split(",")[0].split(":")[1].strip())
                y_value = float(message.split(",")[1].split(":")[1].strip())
                self.get_logger().info(f"提取到的 x 值: {x_value}, y 值: {y_value}")
                return x_value, y_value
            except Exception as e:
                self.get_logger().error(f"解析 response.message 失败: {e}")
                return None, None
        else:
            self.get_logger().error('服务调用失败')
            return None, None  
        
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
        self.get_logger().info('向 /set_position服务发送的坐标为： x, y, z 坐标: {}, {}, {}'.format(x, y, z))
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
def module1(node):
    success = wait_until_position_reached(node,
        target_position=[0, 0, 0.65],position_threshold=0.20,timeout_sec=400.0)  
    return success


# 发送点位并且识别zimu
def module2(node):
    # # 发布目标位置到 /move_base_simple/goal
    node.publish_goal(node.destination[1],position_th=0.1)

    start_time = time.time()
    timeout = 4.0  # 6秒超时
    loop_interval = 0.01  # 每次循环间隔10ms

    while time.time() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=loop_interval)
        node.get_logger().info(f"===================== node.letter_count 为: {node.letter_count} =====================")
        # 可选：精确控制循环间隔
        time.sleep(max(0, loop_interval - (time.time() - start_time)))

    node.aim_letter = node.letter_count.index(max(node.letter_count))  # 获取当前字母数量
    node.get_logger().info(f"===================== 目前aim_letter为: {node.aim_letter} =====================")


# 飞到投放位置
def module3(node):
   # 发布目标位置到 /move_base_simple/goal
    node.publish_goal(node.destination[2],position_th=0.15)

# 操作夹爪 
def module4(node):
    """模块 5：发布 PWM 占空比"""
    duty_cycle = 91  # 设置占空比
    node.publish_pwm_duty_cycle(duty_cycle)
    time.sleep(1)  # 等待 1 秒以确保夹爪动作完成

    # 调用 /set_position 服务 旋转机头,正对击打位置
    success, message = node.call_set_position_service(node.destination[2][0], node.destination[2][1], node.destination[2][2])
    if success:
        node.get_logger().info(f"Module 6: 成功调用 /set_position 服务，消息: {message}")
    else:
        node.get_logger().error(f"Module 6: 调用 /set_position 服务失败，消息: {message}")
    time.sleep(3)  # 等待 4 秒以确保机头旋转成功


# 飞向击打位置
def module5(node,input_value=1):
    """模块 7：飞向击打位置"""
    # 发布目标位置到 /move_base_simple/goal
    node.publish_goal(node.hit_destination[input_value],position_th=0.1)

# 操作激光灯
def module6(node):
    control_gpio(103, value=1)  # 打开激光灯
    time.sleep(2)  # 等待 2 秒
    control_gpio(103, value=0)  # 关闭激光灯


# 控制器进行微调,升高
def module9(node:UAVController,input_value=1):
    # 调用 /set_position 服务
    success, message = node.call_set_position_service(node.hit_destination[input_value][0],node.hit_destination[input_value][1],node.hit_pos_altitude)
    if success:
        node.get_logger().info(f"Module 6: 成功调用 /set_position 服务，消息: {message}")
    else:
        node.get_logger().error(f"Module 6: 调用 /set_position 服务失败，消息: {message}")
    
    wait_until_position_reached(node, target_position=[0.0, 0.0, node.hit_pos_altitude], position_threshold=0.08,direction='z')

        
# 飞回起飞的位置
def module7(node):
    node.get_logger().info("Module 8: 返回起飞位置")
    # 获取初始位置
    init_position = node.get_init_position()
    node.get_logger().info(f"Module 8: 初始位置 x={init_position[0]}, y={init_position[1]}, z=0.4")
    node.publish_goal([init_position[0],init_position[1],0.4])


def main(args=None):
    rclpy.init(args=args)
    # if len(sys.argv) > 1:  # 检查是否有额外参数
    #     try:
    #         # 获取传入的整型参数
    #         input_value = int(sys.argv[1])
    #     except ValueError:
    #         print("Invalid input value. Using default value 0.")


    # 掩码值，决定运行哪个模块
    mask = 0  # 这里设置为运行第一个模块
    node = UAVController()


    while rclpy.ok():
        try:
            # 切换为offboard模式
            if mask == 0:  # 检查掩码值是否启用第一个模块
                result = module0(node)
                if result:
                    logging.getLogger().info("Module 0 executed successfully")
                    logging.getLogger().info(f'{mask}')
                    mask += 1
            # 检测是否起飞
            if mask == 1:
                module1(node)
                logging.getLogger().info("Module 2 executed successfully")
                mask += 1
                # mask = 3

            # # 发送点位往前一点防止撞上
            if mask == 2:
                #发布目标位置到 /move_base_simple/goal
                node.publish_goal(node.destination[0],position_th=0.1)
                logging.getLogger().info("Module 3 executed successfully")
                mask += 1
            # 发送点位并且识别zimu
            if mask == 3:
                module2(node)
                logging.getLogger().info("Module 3 executed successfully")
                mask += 1
                # mask = 6
            # 飞到投放位置
            if mask == 4:
                module3(node)
                time.sleep(2)
                logging.getLogger().info("Module 5 executed successfully")
                mask += 1
            # 夹爪打开，并且掉转机头
            if mask == 5:
                module4(node)
                logging.getLogger().info("Module 6 executed successfully")
                mask += 1
            if mask == 6:
                # 飞向击打位置
                module5(node,node.aim_letter)
                logging.getLogger().info("Module 7 executed successfully")
                mask += 1
            # 控制器进行微调,升高
            if mask == 7:
                module9(node,node.aim_letter)
                time.sleep(2)
                mask += 1
            # 打开激光灯
            if mask == 8:
                node.get_logger().info("Module 6: 操作激光灯")
                module6(node)
                node.get_logger().info("Module 6: 执行完毕")
                mask += 1
            # 飞到投放位置附近
            if mask == 9:
                # 发布目标位置到 /move_base_simple/goal
                node.publish_goal(node.destination[3],position_th=0.15)
                logging.getLogger().info("Module 5 executed successfully")
                mask += 1
                # 飞到字母附近
            if mask == 10:
                # 发布目标位置到 /move_base_simple/goal
                pos = [node.destination[4][0], node.destination[4][1], 0.4]
                # pos = [node.destination[1][0], node.destination[1][1], 0.4]
                node.publish_goal(pos,position_th=0.15)
                logging.getLogger().info("Module 5 executed successfully")
                mask += 1
            # 返回降落点
            if mask == 11:
                module7(node)
                break
                
            rclpy.spin_once(timeout_sec=0.1)  # 处理回调
        except Exception as e:
            logging.getLogger().error(f"An error occurred: {e}")
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
