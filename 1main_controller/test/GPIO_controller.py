#!/usr/bin/env python3
"""
GPIO 控制器
用法：
    sudo python3 GPIO_controller.py              # 使用默认引脚 103，保持 10 秒
    sudo python3 GPIO_controller.py 104 5        # 控制引脚 104，保持 5 秒
"""

import os
import sys
import time

GPIO_BASE = "/sys/class/gpio"


def export_pin(pin: int) -> None:
    """导出指定 GPIO 引脚（如果尚未导出）"""
    gpio_dir = os.path.join(GPIO_BASE, f"gpio{pin}")
    if os.path.exists(gpio_dir):
        print(f"[INFO] GPIO {pin} 已导出，跳过 export")
        return

    export_path = os.path.join(GPIO_BASE, "export")
    try:
        with open(export_path, 'w') as f:
            f.write(str(pin))
        print(f"[INFO] 已导出 GPIO {pin}")
        time.sleep(0.2)  # 等待内核创建 gpio 目录
    except PermissionError:
        print(f"[ERROR] 权限不足，请使用 sudo 运行本脚本")
        sys.exit(1)
    except IOError as e:
        print(f"[ERROR] 无法导出 GPIO {pin}: {e}")
        sys.exit(1)


def set_direction(pin: int, direction: str = "out") -> None:
    """设置引脚方向（in / out）"""
    direction_path = os.path.join(GPIO_BASE, f"gpio{pin}", "direction")
    try:
        with open(direction_path, 'w') as f:
            f.write(direction)
        print(f"[INFO] GPIO {pin} 方向已设为 {direction}")
    except IOError as e:
        print(f"[ERROR] 无法设置 GPIO {pin} 方向: {e}")
        sys.exit(1)


def set_value(pin: int, value: int) -> None:
    """设置引脚电平（0 = 低电平, 1 = 高电平）"""
    value_path = os.path.join(GPIO_BASE, f"gpio{pin}", "value")
    try:
        with open(value_path, 'w') as f:
            f.write(str(value))
        level = "高电平 (ON)" if value else "低电平 (OFF)"
        print(f"[INFO] GPIO {pin} → {level}")
    except IOError as e:
        print(f"[ERROR] 无法设置 GPIO {pin} 值: {e}")
        sys.exit(1)


def main():
    # 解析命令行参数
    pin = int(sys.argv[1]) if len(sys.argv) > 1 else 103
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    print(f"=== GPIO 控制器 ===")
    print(f"引脚: GPIO {pin}")
    print(f"保持时长: {duration} 秒")

    # 1) 导出引脚
    export_pin(pin)

    # 2) 设为输出模式
    set_direction(pin, "out")

    # 3) 输出高电平（打开激光灯）
    set_value(pin, 1)

    # 4) 保持 duration 秒
    print(f"[INFO] 保持 {duration} 秒...")
    time.sleep(duration)

    # 5) 输出低电平（关闭激光灯）
    set_value(pin, 0)

    print("=== 完成 ===")


if __name__ == "__main__":
    main()
