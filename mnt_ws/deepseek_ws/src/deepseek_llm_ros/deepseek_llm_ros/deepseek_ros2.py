#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
from rclpy.parameter import Parameter
import sys
import select

class DeepSeekNode(Node):
    def __init__(self):
        super().__init__('deepseek_node')
        
        # 0 代表没有抽取颜色
        self.declare_parameter('aim_color',0)

        self.lock = threading.Lock()  # 添加线程锁

        # ROS2话题和服务
        self.query_pub = self.create_publisher(String, 'deepseek_response', 10)
        self.query_sub = self.create_subscription(
            String,
            'deepseek_query',
            self.handle_query,
            10)
        
        # 启动DeepSeek进程
        self.deepseek_process = subprocess.Popen(
            ['/home/elf/mnt_ws/deepseek1.5B/install/demo_Linux_aarch64/llm_demo', 
             '/home/elf/mnt_ws/deepseek1.5B/install/demo_Linux_aarch64/rkllmdata/deepseek-1.5b-w8a8-rk3588.rkllm','5000','5000'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
    
        # 启动输出监控线程
        self.output_thread = threading.Thread(target=self.monitor_output)
        self.output_thread.daemon = True
        self.output_thread.start()
        
        # 初始化DeepSeek
        # self.initialize_deepseek()

        # 等待初始化完成
        time.sleep(10)  # 根据实际情况调整等待时间

        # 添加交互式终端线程
        self.interactive_thread = threading.Thread(target=self.interactive_shell)
        self.interactive_thread.daemon = True
        self.interactive_thread.start()

        
    

    def interactive_shell(self):
        """提供交互式终端"""
        print("\nDeepSeek交互终端已启动，输入'quit'退出\n")
        while rclpy.ok():
            try:
                user_input = input("用户> ")
                if user_input.lower() == 'quit':
                    break
                self.write_to_terminater(String(data=user_input))
            except EOFError:
                break


    def initialize_deepseek(self):
        """发送初始化指令给DeepSeek"""
        init_commands = [
            "以下是与AI助手的你是一个运行在机器人上的AI助手。请遵循以下规范:1. 回答简洁明了。2. 专注于机器人相关任务。3. 避免不相关的话题。现在开始交互:",
        ]
        for cmd in init_commands:
            self.deepseek_process.stdin.write(cmd + '\n')
            time.sleep(0.5)  # 给模型一些处理时间
    

    def handle_query(self, msg):
        """处理来自ROS2的查询"""
        self.write_to_terminater(msg)

    def write_to_terminater(self, msg):
        """处理来自ROS2的查询"""
        query = msg.data
        if query.find('红色') != -1 or query.find('red') != -1:
            self.set_parameters([
                Parameter('aim_color', Parameter.Type.INTEGER, 4)
            ])
        elif query.find('绿色') != -1 or query.find('green') != -1:
            self.set_parameters([
                Parameter('aim_color', Parameter.Type.INTEGER, 2)
            ])
        elif query.find('蓝色') != -1 or query.find('blue') != -1:
            self.set_parameters([
                Parameter('aim_color', Parameter.Type.INTEGER, 3)
            ])
        elif query.find('橙色') != -1 or query.find('orange') != -1:
            self.set_parameters([
                Parameter('aim_color', Parameter.Type.INTEGER, 1)
            ])

        self.get_logger().info(f"发送查询: {query}")
        self.deepseek_process.stdin.write(query + '\n')
        self.deepseek_process.stdin.flush()
    

    def _publish_output(self, cleaned):
        """打印并发布清理后的输出"""
        sys.stdout.write("\r")
        sys.stdout.flush()
        print(f"{cleaned}")
        sys.stdout.write("用户> ")
        sys.stdout.flush()

        msg = String()
        msg.data = cleaned
        self.query_pub.publish(msg)

    def monitor_output(self):
        """监控DeepSeek的输出并发布到ROS2话题，过滤 <think>...</think> 思考内容"""
        import re
        skip_lines = [
            "rkllm init start",
            "W rkllm: Warning: Your rknpu driver version is too low, please upgrade to 0.9.7.",
            "I rkllm: rkllm-runtime version: 1.1.4, rknpu driver version: 0.9.6, platform: RK3588",
            "rkllm init success",
            "**********************可输入以下问题对应序号获取回答/或自定义输入********************",
            "[0] 现有一笼子，里面有鸡和兔子若干只，数一数，共有头14个，腿38条，求鸡和兔子各有多少只？",
            "[1] 有28位小朋友排成一行,从左边开始数第10位是学豆,从右边开始数他是第几位?",
            "*************************************************************************"
        ]
        # 匹配 user: / robot: 前缀
        prefix_re = re.compile(r'^(user|robot)\s*:\s*', re.IGNORECASE)
        # 匹配 <think> / </think> 标签（行首或行中）
        think_open_re = re.compile(r'<\s*think\s*>', re.IGNORECASE)
        think_close_re = re.compile(r'<\s*/\s*think\s*>', re.IGNORECASE)

        in_think_block = False  # 是否处于 <think>...</think> 块内

        while rclpy.ok():
            output = self.deepseek_process.stdout.readline()
            if not output.strip():
                continue
            if output.strip() in skip_lines:
                continue

            # 过滤 user: / robot: 前缀
            cleaned = prefix_re.sub('', output).strip()

            if not cleaned:
                continue

            # --- 处理 <think> / </think> 标签 ---
            has_open = think_open_re.search(cleaned)
            has_close = think_close_re.search(cleaned)

            if has_open and has_close:
                # 同一行同时有 <think> 和 </think>：移除中间的思考内容
                cleaned = re.sub(
                    r'<\s*think\s*>.*?<\s*/\s*think\s*>',
                    '', cleaned, flags=re.DOTALL | re.IGNORECASE
                ).strip()
                if cleaned:
                    self._publish_output(cleaned)
            elif has_open:
                # 进入 think 块：输出 <think> 之前的内容（如果有）
                in_think_block = True
                before_think = think_open_re.split(cleaned, maxsplit=1)[0].strip()
                if before_think:
                    self._publish_output(before_think)
            elif has_close:
                # 退出 think 块：输出 </think> 之后的内容（如果有）
                in_think_block = False
                after_think = think_close_re.split(cleaned, maxsplit=1)[-1].strip()
                if after_think:
                    self._publish_output(after_think)
            else:
                # 普通行：不在 think 块内才输出
                if not in_think_block:
                    self._publish_output(cleaned)

    
    def destroy_node(self):
        """清理资源"""
        self.deepseek_process.terminate()
        self.output_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeepSeekNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
