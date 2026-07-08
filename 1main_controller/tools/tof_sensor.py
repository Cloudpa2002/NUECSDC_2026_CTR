"""
tof传感器模块
功能：检测传感器面朝方向上，检测范围内的障碍物距离传感器的距离

---- I2C 权限说明 ----
若遇到 PermissionError: '/dev/i2c-4'，请在目标板（Ubuntu 22.04）上执行
以下命令之一以永久解决权限问题（只需执行一次）：

方法一（推荐）：将当前用户加入 i2c 用户组
    sudo usermod -a -G i2c $USER
    # 然后注销重新登录（或重启）即可生效

方法二：创建 udev 规则，允许所有用户访问 i2c-4
    echo 'KERNEL=="i2c-4", MODE="0666"' | sudo tee /etc/udev/rules.d/99-i2c.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger

临时解决：用当前用户的 Python 以 sudo 运行
    sudo $(which python3) your_script.py
"""

import time
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_vl53l1x

# ---------- 模块级传感器初始化（仅执行一次） ----------
_sensor = None
_init_failed = False  # 标记初始化是否失败，避免重复报错
_last_read_time = 0.0  # 上次成功读取的时间戳，用于控制输出频率


def _init_sensor():
    """初始化 VL53L1X ToF 传感器（内部使用）。"""
    global _sensor, _init_failed
    if _sensor is not None:
        return  # 已初始化
    if _init_failed:
        return  # 已失败过，不再重试
    try:
        i2c = I2C(4)
        _sensor = adafruit_vl53l1x.VL53L1X(i2c, address=0x29)
        _sensor.distance_mode = 1
        _sensor.timing_budget = 100
        _sensor.start_ranging()
    except PermissionError:
        _init_failed = True
        print(
            "[tof_sensor] 无权限访问 /dev/i2c-4。请用以下方式解决：\n"
            "  临时：sudo $(which python3) your_script.py\n"
            "  永久：sudo usermod -a -G i2c $USER （然后重新登录）"
        )


def _get_distance_raw():
    """
    读取传感器原始数据（无频率限制，供内部判断使用）。

    Returns
    -------
    float or None
        测距距离（单位：厘米）；传感器未检测到障碍物或数据未就绪时返回 None。
    """
    global _sensor
    if _sensor is None or not _sensor.data_ready:
        return None
    dist = _sensor.distance
    _sensor.clear_interrupt()
    # distance 可能为 None，或接近最大量程时视为无检测物
    if dist is None or dist >= 400.0:
        return None
    return dist


def get_distance():
    """
    获取当前 ToF 传感器测距值（输出频率限制为 2 Hz）。

    仅在 500ms 采样窗口内返回有效值：
        - 有障碍物 → float（厘米）
        - 无障碍物 → None
    非采样时刻不返回任何有效值（调用方通过 None 判断无新数据）。

    Returns
    -------
    float or None
        测距距离（单位：厘米）。
    """
    global _last_read_time
    if _sensor is None:
        _init_sensor()
    now = time.time()
    if now - _last_read_time < 0.5:  # 未到 2 Hz 采样窗口，不返回有效值
        return None
    _last_read_time = now
    return _get_distance_raw()


def wait_until_closer_than(threshold):
    """
    阻塞等待，直到传感器检测到障碍物进入阈值范围后又离开。

    流程：
        1. 持续监测，直到距离 < threshold（障碍物进入）
        2. 持续监测，直到距离 > threshold（障碍物离开）
        3. 返回 True

    Parameters
    ----------
    threshold : float
        距离阈值（单位：米）。

    Returns
    -------
    bool
        当障碍物进入后又离开阈值范围时返回 True。
        若传感器初始化失败，立即返回 False。
    """
    # 确保传感器已初始化
    _init_sensor()
    if _sensor is None:
        return False

    # 阶段1：等待障碍物进入（距离 < threshold）
    while True:
        dist_cm = _get_distance_raw()
        if dist_cm is not None and dist_cm / 100.0 < threshold:
            break
        time.sleep(0.05)

    # 阶段2：等待障碍物离开（距离 > threshold 或 超出量程）
    # 使用连续计数器避免因传感器测距间隙误判
    _none_count = 0
    while True:
        dist_cm = _get_distance_raw()
        if dist_cm is None:
            _none_count += 1
            if _none_count >= 5:  # 连续 5 次（约 250ms）无检测物才确认离开
                return True
        else:
            _none_count = 0  # 又检测到了，重置计数
            if dist_cm / 100.0 > threshold:
                return True
        time.sleep(0.05)


# ---------- 直接运行脚本时持续打印距离 ----------
if __name__ == "__main__":
    _init_sensor()
    if _sensor is None:
        exit(1)
    while True:
        dist = get_distance()
        if dist is not None:
            print(dist, "cm")
        time.sleep(0.1)
