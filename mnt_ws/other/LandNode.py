import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandTOL

class OffboardLandDemo(Node):
    def __init__(self):
        super().__init__('offboard_land_demo')

        # 1) 切到 AUTO.LAND
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(1.0):
            self.get_logger().info('等待 /mavros/set_mode ...')

        set_mode = SetMode.Request()
        set_mode.base_mode  = 0          # 使用 custom_mode 即可
        set_mode.custom_mode = 'AUTO.LAND'

        self.get_logger().info('切模式 AUTO.LAND...')
        resp = self.set_mode_cli.call_async(set_mode)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result().mode_sent:
            self.get_logger().info('模式切换成功')
        else:
            self.get_logger().error('模式切换失败；尝试直接下 LAND 指令')

        # 2) 保险起见再发一次 cmd/land
        self.land_cli = self.create_client(CommandTOL, '/mavros/cmd/land')
        while not self.land_cli.wait_for_service(1.0):
            self.get_logger().info('等待 /mavros/cmd/land ...')

        req = CommandTOL.Request()
        req.latitude  = float('nan')     # 也可用 0
        req.longitude = float('nan')
        req.altitude  = 0.0
        req.yaw       = 1.57
        req.min_pitch = 0.0

        fut = self.land_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result().success:
            self.get_logger().info('LAND 指令已接受，开始降落')
        else:
            self.get_logger().error(f'LAND 指令被拒绝，MAV_RESULT={fut.result().result}')

        # 节点结束
        rclpy.shutdown()

def main():
    rclpy.init()
    OffboardLandDemo()

if __name__ == '__main__':
    main()
