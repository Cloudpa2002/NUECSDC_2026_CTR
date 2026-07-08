#!/usr/bin/env python3
"""
夹爪舵机控制程序
功能：
    控制夹爪张开到一定角度
用法：
    调用 gripper 函数
"""

from pathlib import Path
import time

PWMCHIP = Path("/sys/class/pwm/pwmchip0")
PWM = PWMCHIP / "pwm0"
PERIOD_NS = 20_000_000  # 20 ms, 50 Hz

# 夹爪打开位置（2ms 脉宽）。实测角度后可调整。
POSITION = 2_000_000

def write_sysfs(path: Path, value: str, ignore_error: bool = False) -> None:
    """
    功能: 向 sysfs 路径写入字符串值，用于控制 PWM 等 sysfs 接口
    输入: path  - sysfs 文件路径（Path 对象）
          value - 要写入的字符串值
          ignore_error - 为 True 时忽略写入异常，默认 False
    输出: 无
    """
    try:
        path.write_text(value)
    except OSError:
        if not ignore_error:
            raise


def setup_pwm() -> None:
    """
    功能: 初始化 PWM 通道，包括导出 pwm0、设置极性、周期和初始占空比，并使能 PWM 输出
    输入: 无
    输出: 无
    """
    if not PWMCHIP.exists():
        raise FileNotFoundError(f"PWM chip not found: {PWMCHIP}")

    if not PWM.exists():
        write_sysfs(PWMCHIP / "export", "0")
        time.sleep(0.2)

    write_sysfs(PWM / "enable", "0", ignore_error=True)
    write_sysfs(PWM / "polarity", "normal", ignore_error=True)
    write_sysfs(PWM / "period", str(PERIOD_NS))
    write_sysfs(PWM / "duty_cycle", "1500000")
    write_sysfs(PWM / "enable", "1")


def set_pulse(pulse_ns: int) -> None:
    """
    功能: 设置 PWM 占空比（脉宽），并打印当前状态
    输入: pulse_ns - 脉宽值，单位纳秒（ns）
    输出: 无
    """
    print(f"占空比={pulse_ns} ns")
    write_sysfs(PWM / "duty_cycle", str(pulse_ns))


def gripper(position: int) -> bool:
    """
    功能: 控制夹爪舵机转动到指定位置并保持
    输入: position - 脉宽值，单位纳秒（ns）
    输出: True  - 执行成功
          False - 执行失败（PWM 芯片不存在）
    """
    if not PWMCHIP.exists():
        print(f"错误：未找到 PWM 芯片 {PWMCHIP}")
        return False

    setup_pwm()
    set_pulse(position)
    print("完成。舵机保持当前位置。")
    return True


def main() -> None:
    """
    功能: 主控制流程：调用 gripper 函数驱动舵机到达 POSITION 位置
    输入: 无
    输出: 无
    """
    success = gripper(POSITION)
    if not success:
        print("夹爪控制失败。")


if __name__ == "__main__":
    main()
