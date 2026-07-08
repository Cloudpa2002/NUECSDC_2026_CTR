#!/usr/bin/env python3
"""
环境监测节点 —— M702 传感器 + 环境分析算法 + DeepSeek R1 蒸馏版报告生成。

架构：
  M702传感器 ──► 环境分析算法(AQI/舒适度/体感温度/建议) ──► DeepSeek R1(自然语言润色) ──► 环境报告
"""

import serial
import time
import re
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from tools.assessment_library import get_assessment_text

# ======================== 硬件配置 ========================
PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

FRAME_HEADER = b"\x3c\x02"
FRAME_LEN = 17

# ======================== DeepSeek 配置 ========================
REPORT_INTERVAL = 10.0       # 生成环境报告的周期（秒）
REPORT_MAX_CHARS = 160       # 报告最大字符数（含气态污染物提示）

# ======================== M702 帧解析 ========================


def parse_frame(frame: bytes):
    """解析 M702 传感器数据帧，返回 (temperature, humidity, pm25, pm10, co2, ch2o, tvoc) 或 None

    M702 帧格式 (17 字节):
      B1-B2: 帧头 0x3C 0x02
      B3-B4: eCO2 (ppm)       B5-B6: eCH2O (μg/m³)
      B7-B8: TVOC (μg/m³)     B9-B10: PM2.5 (μg/m³)
      B11-B12: PM10 (μg/m³)   B13: 温度整数 (bit7=1→负)
      B14: 温度小数/10       B15: 湿度整数
      B16: 湿度小数/10       B17: 校验和 (B1..B16 累加低 8 位)
    """
    checksum = sum(frame[:16]) & 0xFF
    if checksum != frame[16]:
        return None

    co2 = frame[2] * 256 + frame[3]          # eCO2, ppm
    ch2o = frame[4] * 256 + frame[5]         # eCH2O, μg/m³
    tvoc = frame[6] * 256 + frame[7]         # TVOC, μg/m³
    pm25 = frame[8] * 256 + frame[9]         # PM2.5, μg/m³
    pm10 = frame[10] * 256 + frame[11]       # PM10, μg/m³

    temp_int_raw = frame[12]
    temp_decimal = frame[13] / 10.0

    if temp_int_raw & 0x80:
        temperature = -((temp_int_raw & 0x7F) + temp_decimal)
    else:
        temperature = temp_int_raw + temp_decimal

    humidity = frame[14] + frame[15] / 10.0

    return temperature, humidity, pm25, pm10, co2, ch2o, tvoc


# ======================== 环境分析算法层 ========================

# GB 3095-2012 PM2.5 AQI 分指数计算断点
# 格式: (C_low, C_high, I_low, I_high)
_AQI_BP_PM25 = [
    (0,    35,   0,   50),    # 优
    (35,   75,   51,  100),   # 良
    (75,   115,  101, 150),   # 轻度污染
    (115,  150,  151, 200),   # 中度污染
    (150,  250,  201, 300),   # 重度污染
    (250,  500,  301, 500),   # 严重污染
]

# GB 3095-2012 PM10 AQI 分指数计算断点
_AQI_BP_PM10 = [
    (0,    50,    0,   50),   # 优
    (50,   150,   51,  100),  # 良
    (150,  250,  101, 150),   # 轻度污染
    (250,  350,  151, 200),   # 中度污染
    (350,  420,  201, 300),   # 重度污染
    (420,  600,  301, 500),   # 严重污染
]


def _compute_iaqi(concentration: float, breakpoints: list) -> int:
    """通用 IAQI 分指数计算"""
    for c_low, c_high, i_low, i_high in breakpoints:
        if c_low <= concentration <= c_high:
            return int((i_high - i_low) / (c_high - c_low) * (concentration - c_low) + i_low)
    return 500


def compute_aqi_pm25(pm25: float) -> int:
    """根据 PM2.5 浓度（μg/m³）计算 AQI 分指数"""
    return _compute_iaqi(pm25, _AQI_BP_PM25)


def compute_aqi_pm10(pm10: float) -> int:
    """根据 PM10 浓度（μg/m³）计算 AQI 分指数"""
    return _compute_iaqi(pm10, _AQI_BP_PM10)


def compute_overall_aqi(pm25: float, pm10: float) -> int:
    """综合 AQI = max(PM2.5_IAQI, PM10_IAQI)，符合 GB 3095-2012 标准"""
    return max(compute_aqi_pm25(pm25), compute_aqi_pm10(pm10))


def aqi_level(aqi: int) -> str:
    """AQI → 空气质量等级"""
    if aqi <= 50:
        return "优"
    elif aqi <= 100:
        return "良"
    elif aqi <= 150:
        return "轻度污染"
    elif aqi <= 200:
        return "中度污染"
    elif aqi <= 300:
        return "重度污染"
    else:
        return "严重污染"


def temp_level(temp: float) -> str:
    """温度等级判定"""
    if temp < 0:
        return "严寒"
    elif temp < 10:
        return "寒冷"
    elif temp < 18:
        return "偏凉"
    elif temp <= 26:
        return "舒适"
    elif temp <= 32:
        return "偏热"
    else:
        return "炎热"


def humidity_level(humidity: float) -> str:
    """湿度等级判定"""
    if humidity < 30:
        return "干燥"
    elif humidity <= 60:
        return "适中"
    elif humidity <= 80:
        return "偏高"
    else:
        return "潮湿"


def comfort_level(temp: float, humidity: float) -> str:
    """综合温湿度舒适度判定"""
    t = temp_level(temp)
    h = humidity_level(humidity)
    if t == "舒适" and h in ("适中",):
        return "舒适"
    elif t in ("严寒", "寒冷"):
        return "寒冷不适"
    elif t in ("偏热", "炎热") and h in ("偏高", "潮湿"):
        return "闷热不适"
    elif t in ("偏热", "炎热"):
        return "偏热"
    elif h in ("偏高", "潮湿"):
        return "潮湿不适"
    elif h == "干燥":
        return "干燥不适"
    elif t == "偏凉":
        return "偏凉"
    return "一般"


# ======================== 综合评估（调用 tools/assessment_library.py 语句库） ========================


def _simplify_aqi_level(level: str) -> str:
    """将 AQI 等级归并为评估用大类：仅将「严重污染」合并为「重度污染」"""
    return "重度污染" if level == "严重污染" else level


def get_assessment(temperature: float, humidity: float, aqi: int,
                   co2: float = 0, ch2o: float = 0, tvoc: float = 0) -> str:
    """根据温度、湿度、AQI 及气态污染物调用语句库，随机返回一条综合评估"""
    comfort = comfort_level(temperature, humidity)
    aqi_key = _simplify_aqi_level(aqi_level(aqi))
    return get_assessment_text(comfort, aqi_key, co2=co2, ch2o=ch2o, tvoc=tvoc)


# ======================== DeepSeek Prompt 模板 ========================

# 1.5B R1 蒸馏模型无法遵循任何复杂指令。
# 策略：强制一字不差输出，不给模型任何自由发挥空间。
REPORT_PROMPT_TEMPLATE = (
    '请你将以下内容，原封不动，一字不差的进行输出。不允许自行加入任何修饰，必须保证你的输出与以下内容完全一致。'
    '以下是你需要输出的内容：'
    '"[环境报告] {timestamp}时刻：温度{temperature:.1f}°C{temp_lv}，湿度{humidity:.0f}%{hum_lv}，'
    '空气质量{level}，AQI指数{aqi}。综合评估：{assessment}"'
)

# 用于过滤 <think>...</think> 的正则
_THINK_PATTERN = re.compile(r'<\s*think\s*>.*?<\s*/\s*think\s*>', re.DOTALL | re.IGNORECASE)
# 用于提取 [环境报告] 内容
_REPORT_PATTERN = re.compile(r'\[环境报告\]\s*(.+?)(?:\n|$)', re.IGNORECASE)


# ======================== 主节点 ========================

class EnvironmentalMonitor(Node):
    def __init__(self):
        super().__init__('environmental_monitor')

        # ---- 传感器数据（线程安全） ----
        self._sensor_lock = threading.Lock()
        self.latest_sensor = None     # (temperature, humidity, pm25, pm10, co2, ch2o, tvoc)
        self.current_x = None
        self.current_y = None

        # ---- DeepSeek 通信状态 ----
        self._pending_report = False       # 是否有等待回复的报告
        self._report_timestamp = 0.0       # 最近一次完成报告的时间
        self._response_buffer = []         # 累积 DeepSeek 回复行

        # ---- ROS2 底层通信 ----
        self.odom_sub = self.create_subscription(
            Odometry, "Odometry", self.odom_callback, 10)

        # DeepSeek 通信
        self.query_pub = self.create_publisher(String, 'deepseek_query', 10)
        self.response_sub = self.create_subscription(
            String, 'deepseek_response', self.deepseek_callback, 10)

        # 定时器：1Hz CSV 输出 + 环境报告周期触发
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 串口读取线程
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

        self.get_logger().info(
            f"[ENV] M702 sensor @ {PORT} | DeepSeek report every {REPORT_INTERVAL}s"
        )

    # ==================== 传感器数据 ====================

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _serial_reader(self):
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        buffer = bytearray()

        while True:
            data = ser.read(64)
            if data:
                buffer.extend(data)

            while len(buffer) >= FRAME_LEN:
                idx = buffer.find(FRAME_HEADER)
                if idx < 0:
                    buffer.clear()
                    break
                if idx > 0:
                    del buffer[:idx]
                if len(buffer) < FRAME_LEN:
                    break

                frame = bytes(buffer[:FRAME_LEN])
                del buffer[:FRAME_LEN]

                result = parse_frame(frame)
                if result is not None:
                    with self._sensor_lock:
                        self.latest_sensor = result

            time.sleep(0.05)

    def _get_latest_data(self):
        """线程安全地获取最新传感器数据 + 位置"""
        with self._sensor_lock:
            sensor = self.latest_sensor
            x = self.current_x if self.current_x is not None else 0.0
            y = self.current_y if self.current_y is not None else 0.0
        return sensor, x, y

    # ==================== 定时回调 ====================

    def timer_callback(self):
        sensor, x, y = self._get_latest_data()
        if sensor is None:
            return

        temperature, humidity, pm25, pm10, co2, ch2o, tvoc = sensor

        # 1Hz: 原始数据 CSV 输出（x,y 在最后）
        print(f"{temperature:.1f},{humidity:.1f},{pm25},{pm10},{co2},{ch2o},{tvoc},{x:.2f},{y:.2f}", flush=True)

        # 按周期触发 DeepSeek 环境报告
        now = time.time()
        if not self._pending_report and (now - self._report_timestamp >= REPORT_INTERVAL):
            self._trigger_report(temperature, humidity, pm25, pm10, co2, ch2o, tvoc)

    # ==================== 环境报告流程 ====================

    def _trigger_report(self, temperature: float, humidity: float, pm25: float,
                        pm10: float, co2: float, ch2o: float, tvoc: float):
        """
        触发一次环境报告：
        1. 算法层分析
        2. 拼接 Prompt（示例驱动，适配 1.5B 模型）
        3. 发送到 DeepSeek

        注意：调用前调用方已确保 _pending_report == False，
        此处立即置 True 防止在等待回复期间重复发送。
        """
        # 双重保险：如果上一个报告尚未完成，直接跳过
        if self._pending_report:
            self.get_logger().warn("[ENV] 上一个报告尚未完成，跳过本次发送")
            return

        # --- 步骤 1：拼接 Prompt ---
        aqi = compute_overall_aqi(pm25, pm10)
        level = aqi_level(aqi)

        prompt = REPORT_PROMPT_TEMPLATE.format(
            timestamp=time.strftime("%Y-%m-%d %H:%M:%S"),
            assessment=get_assessment(temperature, humidity, aqi,
                                      co2=co2, ch2o=ch2o, tvoc=tvoc),
            temperature=temperature,
            temp_lv=temp_level(temperature),
            humidity=humidity,
            hum_lv=humidity_level(humidity),
            level=level,
            aqi=aqi,
        )

        # --- 步骤 2：发送前锁定状态 ---
        self._pending_report = True
        self._response_buffer = []  # 清空累积缓冲区

        msg = String()
        msg.data = prompt
        self.query_pub.publish(msg)

    def deepseek_callback(self, msg: String):
        """接收 DeepSeek 回复，累积直到收到完整报告"""
        if not self._pending_report:
            return

        line = msg.data.strip()
        if not line:
            return

        # 累积每一行回复
        self._response_buffer.append(line)

        # 检查是否已经收到 [环境报告]
        full_text = '\n'.join(self._response_buffer)
        if '[环境报告]' in full_text:
            # 收到完整报告，立即处理
            self._process_accumulated_response(full_text)

    def _process_accumulated_response(self, full_text: str):
        """从累积的回复文本中提取最终报告（只接受 DeepSeek 输出，不做兜底）"""
        # --- 1. 剥离 <think>...</think> ---
        clean_text = _THINK_PATTERN.sub('', full_text).strip()

        # --- 2. 提取 [环境报告] 内容 ---
        match = _REPORT_PATTERN.search(clean_text)
        if match:
            report_text = match.group(1).strip()
            if len(report_text) > REPORT_MAX_CHARS:
                report_text = report_text[:REPORT_MAX_CHARS]
            final_report = f"[环境报告] {report_text}"
            self._finalize_report(final_report)
        else:
            # 未匹配到 [环境报告] → 放弃，不输出任何内置兜底文本
            self.get_logger().warn(
                f"[ENV] DeepSeek 回复中未找到 [环境报告]，已丢弃。原始回复: {clean_text[:80]}..."
            )
            self._abort_report()

    def _abort_report(self):
        """放弃本次报告：只重置状态，不输出任何内容"""
        self._pending_report = False
        self._report_timestamp = time.time()

    def _finalize_report(self, report: str):
        """完成报告输出并清理状态"""
        self._pending_report = False
        # 从收到回复这一刻重新计时，确保下一次报告间隔完整
        self._report_timestamp = time.time()

        # stdout 输出供可视化仪表盘解析（DEEPSEEK_REPORT: 前缀）
        print(f"DEEPSEEK_REPORT:{report}", flush=True)


# ======================== 入口 ========================

def main():
    rclpy.init()
    node = EnvironmentalMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

