#!/usr/bin/env python3
"""
环境传感器数据模拟器 —— 4个函数，1Hz 频率输出到终端

函数清单:
  1. gen_temperature()   — 日常温度范围随机数
  2. gen_humidity()      — 日常湿度范围随机数
  3. gen_pm25()          — PM2.5 指数范围随机数
  4. gen_xy()            — 连续随机 XY 坐标（随机游走）
"""

import random
import time
from datetime import datetime


# ────────────────────────────────────────
#  四个核心随机数生成函数
# ────────────────────────────────────────

def gen_temperature() -> float:
    """
    生成日常生活中的温度随机数
    范围: -10.0°C ~ 45.0°C（涵盖冬季严寒 → 夏季酷暑）
    """
    return round(random.uniform(-10.0, 45.0), 1)


def gen_humidity() -> float:
    """
    生成日常生活中的湿度随机数
    范围: 20.0% ~ 95.0%（涵盖干燥 → 潮湿）
    """
    return round(random.uniform(20.0, 95.0), 1)


def gen_pm25() -> float:
    """
    生成日常生活中的 PM2.5 指数随机数
    范围: 5.0 ~ 300.0 μg/m³
        < 50     优
        50-100   良
        100-150  轻度污染
        150-200  中度污染
        200-300  重度污染
    """
    return round(random.uniform(5.0, 300.0), 1)


# ── XY 坐标（随机游走，保证连续性）──
_xy_state = {"x": 0.0, "y": 0.0}


def gen_xy() -> tuple:
    """
    生成连续随机 XY 坐标（随机游走）
    每次在上一位置基础上做小幅随机偏移，模拟连续轨迹
    返回: (x: float, y: float)
    """
    step = 0.5  # 每次最大步长
    _xy_state["x"] += round(random.uniform(-step, step), 2)
    _xy_state["y"] += round(random.uniform(-step, step), 2)
    return round(_xy_state["x"], 2), round(_xy_state["y"], 2)


# ────────────────────────────────────────
#  主循环 —— 1Hz 打印到终端
# ────────────────────────────────────────

def main():
    print("=" * 70)
    print("  环境数据模拟器启动  |  频率: 1 Hz  |  按 Ctrl+C 停止")
    print("=" * 70)
    print(f"{'时间':^12} {'温度(°C)':>8} {'湿度(%)':>8} {'PM2.5':>8} {'X坐标':>8} {'Y坐标':>8}")
    print("-" * 70)

    try:
        while True:
            t = gen_temperature()
            h = gen_humidity()
            p = gen_pm25()
            x, y = gen_xy()

            now = datetime.now().strftime("%H:%M:%S")
            print(f"{now:>12} {t:>8.1f} {h:>8.1f} {p:>8.1f} {x:>8.2f} {y:>8.2f}")

            time.sleep(1.0)  # 1Hz
    except KeyboardInterrupt:
        print("\n已停止。")


if __name__ == "__main__":
    main()
