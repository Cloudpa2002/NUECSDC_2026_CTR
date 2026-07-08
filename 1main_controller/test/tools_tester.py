#!/usr/bin/env python3
"""
tools测试脚本 用于测试tools中的功能函数
"""
import time
import sys
import os

# 将父目录加入模块搜索路径，以便使用 from tools.xxx 导入
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from tools.gripper_controller import gripper
from tools.letters_detector import letters_detector
from tools.gpio_controller import single_shot_level_controller
from tools.ground_circle_detector import ground_circle_detector, ground_circle_detector1
from tools.tof_sensor import get_distance, wait_until_closer_than

if __name__ == '__main__':
    print("启动测试")
    result1 = wait_until_closer_than(1.0)

    print(f"被测函数返回值: {result1}")

    print("测试结束。")




