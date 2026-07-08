#!/usr/bin/env python3
"""
调用 C++ yolov5_videocapture_demo 进行字母识别，
通过 ROS2 订阅 /yolov5_results 话题，在终端打印识别到的字母。
"""

import os
import subprocess
import signal
import time
import argparse
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ==================== 默认配置 ====================

# 默认的 C++ 可执行文件路径
DEFAULT_DEMO_PATH = (
    "/home/elf/mnt_ws/lubancat_ai_manual_code/example/yolov5/cpp/install"
    "/rk3588_linux/yolov5_videocapture_demo"
)

# 默认的 RKNN 模型路径
DEFAULT_MODEL_PATH = (
    "/home/elf/mnt_ws/lubancat_ai_manual_code/example/yolov5/cpp/install"
    "/abc_qz.rknn"
)

# 默认摄像头编号
DEFAULT_CAMERA_INDEX = 23

# ROS2 话题名（与 C++ 程序发布的话题保持一致）
RESULT_TOPIC = "/yolov5_results"

# ================================================


# ==================== 子进程管理 ====================

class SubprocessRunner:
    """
    功能: 以子进程方式启动 C++ yolov5_videocapture_demo，并管理其生命周期
    """

    def __init__(self, demo_path: str, model_path: str, camera_index: int,
                 freeze_timeout: float = 8.0):
        """
        功能: 初始化子进程启动器
        输入: demo_path      - C++ 可执行文件路径
              model_path     - RKNN 模型文件路径
              camera_index   - 摄像头设备编号
              freeze_timeout - 输出静默超过此秒数则判定为冻结，默认 8 秒
        输出: 无
        """
        self.demo_path = demo_path
        self.model_path = model_path
        self.camera_index = camera_index
        self.freeze_timeout = freeze_timeout
        self.process = None
        self._last_output_time = 0.0
        self._last_progress_time = 0.0  # 最后"有意义进展"的时间
        self._output_lines = []  # 缓存最近输出行
        self._frozen = False
        self._rga_error_count = 0  # RGA 噪音行计数
        self._rknn_run_count = 0   # 推理次数计数
        self._last_status_time = 0.0  # 上次打印状态摘要的时间
        # 继承当前进程的环境变量，确保 X11/图形界面正常
        self._env = os.environ.copy()
        # 确保 DISPLAY 存在（GUI 显示必须）
        if "DISPLAY" not in self._env:
            self._env["DISPLAY"] = ":0"

    # 已知的 RGA/Rockchip 噪音输出模式（这些不表示程序有进展）
    _NOISE_PATTERNS = [
        "RgaCollorFill",
        "RGA_COLORFILL",
        "im2d_rga_impl rga_task_submit",
        "im2d_rga_impl rga_dump_channel_info",
        "im2d_rga_impl rga_dump_opt",
        "Failed to call RockChipRga",
        "rect[x,y,w,h]",
        "image[w,h,ws,hs,f]",
        "buffer[handle,fd,va,pa]",
        "color_space =",
        "global_alpha =",
        "set_core[",
        "color[0x",
        "acquir_fence",
        "fill dst image",
        "rga_api version",
    ]

    # 有意义的进展行模式
    _PROGRESS_PATTERNS = [
        "rknn_run",
        "load lable",
        "model input",
        "output tensors",
        "scale=",
        "[WARN]",
        "[ERROR]",
        "cap1",
        "cap2",
        "camera",
        "Camera",
    ]

    @staticmethod
    def _is_noise(line: str) -> bool:
        """判断一行输出是否为 RGA 硬件噪音"""
        for pat in SubprocessRunner._NOISE_PATTERNS:
            if pat in line:
                return True
        return False

    @staticmethod
    def _is_progress(line: str) -> bool:
        """判断一行输出是否表示实际进展"""
        for pat in SubprocessRunner._PROGRESS_PATTERNS:
            if pat in line:
                return True
        return False

    def _read_output(self) -> str:
        """
        功能: 非阻塞读取子进程 stdout 一行，分类并更新计时器
              噪音行不重置进展计时器，有意义行才重置
        输入: 无
        输出: str - 读取到的行（可能为空字符串）
        """
        if self.process is None or self.process.stdout is None:
            return ""
        try:
            ready, _, _ = select.select([self.process.stdout], [], [], 0.0)
            if ready:
                line = self.process.stdout.readline()
                if line:
                    now = time.time()
                    stripped = line.strip()
                    self._last_output_time = now

                    # 噪音行：只计数不打印，不重置进展计时器
                    if self._is_noise(stripped):
                        self._rga_error_count += 1
                        return ""  # 不向外部返回噪音行

                    # 有意义进展行：重置进展计时器
                    if self._is_progress(stripped):
                        self._last_progress_time = now
                        if "rknn_run" in stripped:
                            self._rknn_run_count += 1

                    self._output_lines.append(stripped)
                    if len(self._output_lines) > 20:
                        self._output_lines.pop(0)
                    return line
        except (ValueError, OSError):
            pass
        return ""

    def check_frozen(self) -> bool:
        if self._last_progress_time == 0.0:
            if self._last_output_time == 0.0:
                return False
            elapsed = time.time() - self._last_output_time
        else:
            elapsed = time.time() - self._last_progress_time
        return elapsed > self.freeze_timeout

    def start(self) -> bool:
        if not os.path.isfile(self.demo_path):
            return False
        if not os.path.isfile(self.model_path):
            return False

        cmd = [self.demo_path, self.model_path, str(self.camera_index)]

        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
                env=self._env,
            )
            self._last_output_time = time.time()
            return True
        except (FileNotFoundError, PermissionError, Exception):
            return False

    def stop(self, force: bool = False) -> None:
        if self.process is None:
            return
        try:
            if force:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            else:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=0 if force else 5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait()
        self.process = None

    def is_running(self) -> bool:
        """检查子进程是否仍在运行"""
        return self.process is not None and self.process.poll() is None


# ==================== ROS2 包装节点 ====================

class LettersTesterNode(Node):
    """
    功能: ROS2 节点，订阅 /yolov5_results 话题，记录并打印识别到的字母
    """

    def __init__(self):
        super().__init__("letters_tester_node")

        self.latest_result = "暂无"
        self.detected_letters = []  # 收集所有识别到的字母
        self.detection_count = 0
        self.result_sub = self.create_subscription(
            String, RESULT_TOPIC, self._result_callback, 10
        )

    def _result_callback(self, msg: String):
        letter = msg.data
        self.latest_result = letter
        self.detected_letters.append(letter)
        self.detection_count += 1
        # 用醒目的格式打印
        print(f"  >>> 识别到字母: {letter} <<<  (累计 {self.detection_count} 次)")


# ==================== 主入口 ====================

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="字母识别测试脚本 —— 调用 C++ yolov5_videocapture_demo 进行字母识别",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python3 letters_tester.py
  python3 letters_tester.py --camera 23
  python3 letters_tester.py --run-seconds 20 --camera 23
  python3 letters_tester.py --run-detections 3 --camera 23
        """,
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=DEFAULT_CAMERA_INDEX,
        help=f"摄像头设备编号（默认: {DEFAULT_CAMERA_INDEX}）",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help=f"RKNN 模型路径（默认: {DEFAULT_MODEL_PATH}）",
    )
    parser.add_argument(
        "--demo",
        type=str,
        default=DEFAULT_DEMO_PATH,
        help=f"C++ 可执行文件路径（默认: {DEFAULT_DEMO_PATH}）",
    )
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=6.0,
        help="C++ 程序运行指定秒数后自动退出（默认: 6 秒）。设为 0 表示持续运行直到手动 Ctrl+C",
    )
    parser.add_argument(
        "--run-detections",
        type=int,
        default=0,
        help="收到指定次数识别结果后自动退出（默认: 0 表示不限制）。"
             "与 --run-seconds 同时设置时，任一条件满足即退出",
    )
    return parser.parse_args()


def _run_detection(demo_path, model_path, camera_index, run_seconds, run_detections):
    """
    核心检测逻辑：启动 C++ 子进程，收集识别结果，返回最终判定的字母。
    输入: demo_path, model_path, camera_index, run_seconds, run_detections
    输出: str - 识别到的字母（频次最高者）；未识别到则返回 ""
    """
    if not rclpy.ok():
        rclpy.init()
    node = LettersTesterNode()

    _shutdown_called = False
    _algo_start_time = 0.0

    def safe_shutdown():
        nonlocal _shutdown_called
        if _shutdown_called:
            return
        _shutdown_called = True
        try:
            node.destroy_node()
        except Exception:
            pass
        # 不再调用 rclpy.shutdown()，生命周期由主脚本管理

    def should_exit() -> bool:
        if run_seconds > 0 and _algo_start_time > 0:
            if time.time() - _algo_start_time >= run_seconds:
                return True
        if run_detections > 0:
            if node.detection_count >= run_detections:
                return True
        return False

    runner = SubprocessRunner(
        demo_path=demo_path,
        model_path=model_path,
        camera_index=camera_index,
    )

    if not runner.start():
        safe_shutdown()
        return ""

    _algo_start_time = time.time()

    try:
        while rclpy.ok() and runner.is_running():
            runner._read_output()
            rclpy.spin_once(node, timeout_sec=0.1)

            now = time.time()
            if now - runner._last_status_time > 10.0:
                runner._last_status_time = now

            if should_exit():
                runner.stop(force=True)
                break

            if runner.check_frozen():
                runner.stop(force=True)
                break

    except KeyboardInterrupt:
        pass
    finally:
        if runner.is_running():
            runner.stop()

    # 汇总：返回出现次数最多的字母
    result = ""
    if node.detected_letters:
        from collections import Counter
        stats = Counter(node.detected_letters)
        result = stats.most_common(1)[0][0]

    safe_shutdown()
    return result


def letters_detector(camera_index: int = 23):
    """
    供外部调用的函数。
    输入: camera_index - 摄像头设备编号，默认 23
    输出: str - 识别到的字母（频次最高者）；未识别到则返回 ""

    用法:
        from tools.letters_detector import letters_detector
        letter = letters_detector()          # 使用默认摄像头 23
        letter = letters_detector(0)         # 使用摄像头 0
    """
    return _run_detection(
        demo_path=DEFAULT_DEMO_PATH,
        model_path=DEFAULT_MODEL_PATH,
        camera_index=camera_index,
        run_seconds=3.0,
        run_detections=0,
    )


def main():
    """命令行入口"""
    args = parse_args()
    result = _run_detection(
        demo_path=args.demo,
        model_path=args.model,
        camera_index=args.camera,
        run_seconds=args.run_seconds,
        run_detections=args.run_detections,
    )
    if result:
        print(f"\n最终判定: {result}")
    else:
        print("\n未识别到字母")


if __name__ == "__main__":
    main()


