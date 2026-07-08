#!/usr/bin/env python3
"""
GPIO 控制器 
功能：
    将指定GPIO引脚置为高电平，并在指定时间长度后变为低电平
用法：
    调用 gpio_controller 函数
"""

import os
import time

GPIO_BASE = "/sys/class/gpio"


def _export_pin(pin: int) -> bool:
    """导出指定 GPIO 引脚（如果尚未导出）"""
    gpio_dir = os.path.join(GPIO_BASE, f"gpio{pin}")
    if os.path.exists(gpio_dir):
        return True

    export_path = os.path.join(GPIO_BASE, "export")
    try:
        with open(export_path, 'w') as f:
            f.write(str(pin))
        time.sleep(0.2)  # 等待内核创建 gpio 目录
        return True
    except PermissionError:
        print(f"[ERROR] 权限不足，请使用 sudo 运行本脚本")
        return False
    except IOError as e:
        print(f"[ERROR] 无法导出 GPIO {pin}: {e}")
        return False


def _set_direction(pin: int, direction: str = "out") -> bool:
    """设置引脚方向（in / out）"""
    direction_path = os.path.join(GPIO_BASE, f"gpio{pin}", "direction")
    try:
        with open(direction_path, 'w') as f:
            f.write(direction)
        return True
    except IOError as e:
        print(f"[ERROR] 无法设置 GPIO {pin} 方向: {e}")
        return False


def _set_value(pin: int, value: int) -> bool:
    """设置引脚电平（0 = 低电平, 1 = 高电平）"""
    value_path = os.path.join(GPIO_BASE, f"gpio{pin}", "value")
    try:
        with open(value_path, 'w') as f:
            f.write(str(value))
        return True
    except IOError as e:
        print(f"[ERROR] 无法设置 GPIO {pin} 值: {e}")
        return False


def single_shot_level_controller(pin: int, level: int) -> bool:
    """
    将指定 GPIO 引脚置为高电平或低电平（单次操作，不自动恢复）。

    参数:
        pin:   GPIO 引脚编号（如 103 表示 GPIO103）
        level: 目标电平，0 = 低电平，1 = 高电平

    返回:
        True:  操作成功
        False: 操作失败
    """
    # 1) 导出引脚
    if not _export_pin(pin):
        return False

    # 2) 设为输出模式
    if not _set_direction(pin, "out"):
        return False

    # 3) 输出目标电平
    if not _set_value(pin, level):
        return False

    return True


def gpio_controller(pin: int, duration: float) -> bool:
    """
    控制指定 GPIO 引脚输出高电平并保持一段时间后恢复低电平。

    参数:
        pin:      GPIO 引脚编号
        duration: 高电平保持时长（秒）

    返回:
        True:  操作成功
        False: 操作失败
    """
    # 1) 导出引脚
    if not _export_pin(pin):
        return False

    # 2) 设为输出模式
    if not _set_direction(pin, "out"):
        return False

    # 3) 输出高电平
    if not _set_value(pin, 1):
        return False

    # 4) 保持 duration 秒
    time.sleep(duration)

    # 5) 输出低电平
    if not _set_value(pin, 0):
        return False

    return True


if __name__ == "__main__":
    print("[ERROR] 本脚本不支持直接运行，请作为模块导入使用：")
    print("  from gpio_controller import gpio_controller")
    print("  success = gpio_controller(103, 5.0)")
