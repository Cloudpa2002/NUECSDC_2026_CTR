#!/usr/bin/env python3
"""
夹爪舵机控制程序
"""

from pathlib import Path
import time

PWMCHIP = Path("/sys/class/pwm/pwmchip0")
PWM = PWMCHIP / "pwm0"
PERIOD_NS = 20_000_000  # 20 ms, 50 Hz

# 夹爪打开位置（2ms 脉宽）。实测角度后可调整。
POSITION = 1800000



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


def set_pulse(label: str, pulse_ns: int) -> None:
    """
    功能: 设置 PWM 占空比（脉宽），并打印当前状态
    输入: label    - 操作标签，用于终端打印标识
          pulse_ns - 脉宽值，单位纳秒（ns）
    输出: 无
    """
    print(f"{label}：占空比={pulse_ns} ns")
    write_sysfs(PWM / "duty_cycle", str(pulse_ns))


def main() -> None:
    """
    功能: 主控制流程：初始化 PWM，驱动舵机到达目标位置并保持
    输入: 无
    输出: 无
    """
    setup_pwm()
    set_pulse("夹爪打开", POSITION)


if __name__ == "__main__":
    main()
