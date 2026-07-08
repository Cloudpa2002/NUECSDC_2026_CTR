#!/usr/bin/env python3
"""
Qt5 可视化界面脚本 —— 显示脚本内某个函数的输出

用法:
    python3 qt5_gui_viewer.py

也可以从 stdin 读取数据（兼容 original consumer.py 的管道用法）:
    echo "3.14" | python3 qt5_gui_viewer.py
    ssh elf@192.168.43.226 'python3 -u /home/elf/1main_controller/test/publisher.py' | python3 -u qt5_gui_viewer.py
"""

import sys
import threading
import time
import random
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTextEdit, QPushButton, QLabel, QLineEdit, QGroupBox, QSplitter
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QTextCursor


# ──────────────────────────────────────────────
# 核心业务函数：你可以把任意逻辑放在这里
# ──────────────────────────────────────────────
def compute_square(value: float) -> str:
    """对输入值做平方计算（示例函数）"""
    result = value * value
    output = f"[{datetime.now().strftime('%H:%M:%S')}] 输入: {value:.4f}  →  输出: {result:.4f}"
    return output


def compute_stats(values: list) -> str:
    """对一组值做统计（示例函数，展示多行输出）"""
    if not values:
        return "无数据"
    lines = [
        f"计数: {len(values)}",
        f"总和: {sum(values):.4f}",
        f"均值: {sum(values)/len(values):.4f}",
        f"最大: {max(values):.4f}",
        f"最小: {min(values):.4f}",
    ]
    return "\n".join(lines)


# ──────────────────────────────────────────────
# 信号桥接对象（用于线程安全地更新 GUI）
# ──────────────────────────────────────────────
class SignalBridge(QObject):
    """将后台线程的输出安全地传递到主线程 GUI"""
    new_output = pyqtSignal(str)


# ──────────────────────────────────────────────
# 管道输入监听线程
# ──────────────────────────────────────────────
class StdinReader(threading.Thread):
    """从 stdin 读取数据，通过信号发送到 GUI"""

    def __init__(self, bridge: SignalBridge):
        super().__init__(daemon=True)
        self.bridge = bridge

    def run(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                value = float(line)
                self.bridge.new_output.emit(compute_square(value))
            except ValueError:
                self.bridge.new_output.emit(f"[错误] 无法解析: {line}")


# ──────────────────────────────────────────────
# 定时器线程（定期生成模拟数据）
# ──────────────────────────────────────────────
class TimedDataGenerator(QObject):
    """用 QTimer 定期调用函数并输出结果"""
    new_output = pyqtSignal(str)
    stats_ready = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._history = []

    def generate_one(self):
        """生成一个随机数并调用 compute_square"""
        value = round(random.uniform(-10, 10), 4)
        self._history.append(value)
        return compute_square(value)

    def generate_stats(self):
        """计算历史数据的统计"""
        return compute_stats(self._history)

    def clear_history(self):
        self._history.clear()
        return "[已清空历史数据]"


# ──────────────────────────────────────────────
# 主窗口
# ──────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt5 函数输出可视化")
        self.setMinimumSize(680, 540)

        # 信号桥
        self.bridge = SignalBridge()
        self.bridge.new_output.connect(self._append_output)

        # 定时数据生成器
        self.generator = TimedDataGenerator()
        self.generator.new_output.connect(self._append_output)
        self.generator.stats_ready.connect(self._show_stats)

        # 定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer_tick)

        # stdin 线程
        self.stdin_reader = StdinReader(self.bridge)

        self._init_ui()

    # ── UI 初始化 ──────────────────────────────
    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)

        # ---------- 顶部控制区 ----------
        control_group = QGroupBox("控制面板")
        control_layout = QHBoxLayout(control_group)

        # 输入框
        control_layout.addWidget(QLabel("输入数值:"))
        self.input_edit = QLineEdit()
        self.input_edit.setPlaceholderText("输入数字后回车...")
        self.input_edit.returnPressed.connect(self._on_manual_input)
        control_layout.addWidget(self.input_edit)

        # 手动触发按钮
        btn_manual = QPushButton("计算平方")
        btn_manual.clicked.connect(self._on_manual_input)
        control_layout.addWidget(btn_manual)

        # 自动定时按钮
        self.btn_auto = QPushButton("▶ 开始定时生成")
        self.btn_auto.setCheckable(True)
        self.btn_auto.toggled.connect(self._on_toggle_auto)
        control_layout.addWidget(self.btn_auto)

        # 统计按钮
        btn_stats = QPushButton("显示统计")
        btn_stats.clicked.connect(self._on_show_stats)
        control_layout.addWidget(btn_stats)

        # 清空按钮
        btn_clear = QPushButton("清空")
        btn_clear.clicked.connect(self._on_clear)
        control_layout.addWidget(btn_clear)

        root_layout.addWidget(control_group)

        # ---------- 输出区 ----------
        splitter = QSplitter(Qt.Vertical)

        # 实时输出
        output_group = QGroupBox("函数输出（实时）")
        output_layout = QVBoxLayout(output_group)
        self.output_view = QTextEdit()
        self.output_view.setReadOnly(True)
        self.output_view.setFont(QFont("Courier New", 10))
        self.output_view.setStyleSheet("background-color: #1e1e1e; color: #00ff88;")
        output_layout.addWidget(self.output_view)
        splitter.addWidget(output_group)

        # 统计输出
        stats_group = QGroupBox("统计输出")
        stats_layout = QVBoxLayout(stats_group)
        self.stats_view = QTextEdit()
        self.stats_view.setReadOnly(True)
        self.stats_view.setFont(QFont("Courier New", 10))
        self.stats_view.setStyleSheet("background-color: #1e1e22; color: #ffcc00;")
        stats_layout.addWidget(self.stats_view)
        splitter.addWidget(stats_group)

        splitter.setSizes([300, 150])
        root_layout.addWidget(splitter)

        # 状态栏
        self.statusBar().showMessage("就绪 | 可直接输入数值，或启动定时生成")

    # ── 槽函数 ────────────────────────────────
    def _append_output(self, text: str):
        """追加一行到输出区（线程安全）"""
        self.output_view.append(text)
        # 自动滚动到底部
        self.output_view.moveCursor(QTextCursor.End)

    def _show_stats(self, text: str):
        self.stats_view.setPlainText(text)

    def _on_manual_input(self):
        raw = self.input_edit.text().strip()
        if not raw:
            return
        try:
            value = float(raw)
            result = compute_square(value)
            self.generator._history.append(value)
            self._append_output(result)
            self.input_edit.clear()
        except ValueError:
            self._append_output(f"[错误] 无效输入: {raw}")

    def _on_toggle_auto(self, checked: bool):
        if checked:
            self.timer.start(800)  # 每 800ms 生成一条
            self.btn_auto.setText("⏸ 停止定时生成")
            self.statusBar().showMessage("定时生成运行中...")
        else:
            self.timer.stop()
            self.btn_auto.setText("▶ 开始定时生成")
            self.statusBar().showMessage("定时生成已停止")

    def _on_timer_tick(self):
        result_line = self.generator.generate_one()
        self._append_output(result_line)

    def _on_show_stats(self):
        stats = self.generator.generate_stats()
        self._show_stats(stats)

    def _on_clear(self):
        self.output_view.clear()
        self.generator.clear_history()
        self.stats_view.clear()
        self.statusBar().showMessage("已清空")

    # ── 关闭事件 ────────────────────────────
    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


# ──────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 全局样式：暗色主题
    app.setStyleSheet("""
        QMainWindow { background-color: #2b2b2b; }
        QGroupBox {
            font-weight: bold; color: #cccccc;
            border: 1px solid #555; border-radius: 4px; margin-top: 10px; padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin; left: 10px; padding: 0 4px;
        }
        QPushButton {
            background-color: #3c3c3c; color: #eee; border: 1px solid #555;
            padding: 5px 14px; border-radius: 3px;
        }
        QPushButton:hover { background-color: #505050; }
        QPushButton:checked { background-color: #0078d4; }
        QLineEdit {
            background-color: #3c3c3c; color: #eee; border: 1px solid #555;
            padding: 4px; border-radius: 3px;
        }
        QLabel { color: #ccc; }
        QStatusBar { color: #aaa; }
    """)

    window = MainWindow()
    window.show()

    # 启动 stdin 监听线程
    window.stdin_reader.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
