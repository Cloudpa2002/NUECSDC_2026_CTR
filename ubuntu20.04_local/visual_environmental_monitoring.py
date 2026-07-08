#!/usr/bin/env python3
# 环境传感器数据可视化仪表盘 —— 通过 Qt5 实时展示远程传感器数据

"""
基于 Qt5 的可视化仪表盘 —— 实时展示 stdin 管道传入的传感器数据
  时间 | 温度 | 湿度 | PM2.5 | PM10 | CO2 | CH2O | TVOC | XY 坐标（地图视图）

数据格式（stdin 每行）: 温度,湿度,PM2.5,PM10,CO2,CH2O,TVOC,x,y
示例: 25.3,60.1,35.0,50.0,800,30,200,2.5,-1.3

地图说明：
  - 高亮方块由无人机当前坐标所在的 1×1 整数网格动态决定
  - 蓝色轨迹线显示最近 20 个历史位置

运行方式:
ssh elf@192.168.43.226  'source /opt/ros/humble/setup.bash && python3 -u /home/elf/1main_controller/environmental_monitoring.py'  |  python3 -u visual_environmental_monitoring.py
"""

import math
import os
import sys
import collections
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout,
    QLabel, QGroupBox, QStatusBar, QFrame, QHBoxLayout, QSizePolicy,
    QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal, QSocketNotifier, QObject
from PyQt5.QtGui import (
    QFont, QColor, QPainter, QPen, QBrush, QPainterPath, QPolygonF, QMouseEvent
)

# 数据来源：stdin 管道（远程计算板输出 "温度,湿度,PM2.5,PM10,CO2,CH2O,TVOC,x,y" 行）


# ==================== 温度颜色工具 ====================

# 9 档温度颜色：从正蓝 (#0000ff) 渐变到正红 (#ff0000)
# 8 档温度颜色：正蓝 → 浅蓝 → 橙 → 正红
_TEMP_COLORS = [
    "#0000ff",  # 0: 正蓝   -10.0°C
    "#006aff",  # 1: 蓝      -2.1°C
    "#00aaff",  # 2: 浅蓝     5.7°C
    "#00ccbb",  # 3: 蓝绿    13.6°C
    "#55cc00",  # 4: 绿      21.4°C
    "#ffaa00",  # 5: 橙      29.3°C
    "#ff5500",  # 6: 橙红    37.1°C
    "#ff0000",  # 7: 正红    45.0°C
]


def get_temp_color(temp: float) -> str:
    """
    功能: 根据温度值返回 8 档颜色中的对应颜色
          温度范围 [-10, 45] 线性映射到 0~7 共 8 档
    输入: temp - float，温度值
    输出: str - CSS 十六进制颜色字符串
    """
    T_MIN, T_MAX = -10.0, 45.0
    clamped = max(T_MIN, min(T_MAX, temp))
    idx = int(round((clamped - T_MIN) / (T_MAX - T_MIN) * (len(_TEMP_COLORS) - 1)))
    return _TEMP_COLORS[max(0, min(len(_TEMP_COLORS) - 1, idx))]


# 6 档湿度颜色：干燥(棕) → 适中(青) → 极湿(深蓝)
_HUMI_COLORS = [
    "#8b7355",  # 0: 干燥     0%
    "#c9a96e",  # 1: 偏干    20%
    "#7ec8e3",  # 2: 适中    40%
    "#4fc3f7",  # 3: 偏湿    60%
    "#1e88e5",  # 4: 潮湿    80%
    "#0d47a1",  # 5: 极湿   100%
]


def get_humi_color(humi: float) -> str:
    """根据湿度值 [0, 100] 返回 6 档颜色"""
    clamped = max(0.0, min(100.0, humi))
    idx = int(round(clamped / 100.0 * (len(_HUMI_COLORS) - 1)))
    return _HUMI_COLORS[max(0, min(len(_HUMI_COLORS) - 1, idx))]


def get_pm25_color(pm25: float) -> str:
    """PM2.5 按 AQI 等级返回颜色"""
    if pm25 <= 50:
        return "#66bb6a"
    elif pm25 <= 100:
        return "#ffb74d"
    elif pm25 <= 150:
        return "#ff8a65"
    elif pm25 <= 200:
        return "#ef5350"
    else:
        return "#ab47bc"


def get_pm10_color(pm10: float) -> str:
    """PM10 按 AQI 等级返回颜色"""
    if pm10 <= 50:
        return "#43a047"
    elif pm10 <= 150:
        return "#ff9800"
    elif pm10 <= 250:
        return "#ff7043"
    elif pm10 <= 350:
        return "#e53935"
    else:
        return "#8e24aa"


def get_co2_color(co2: float) -> str:
    """CO2 按浓度返回颜色"""
    if co2 <= 800:
        return "#66bb6a"
    elif co2 <= 1000:
        return "#ffb74d"
    elif co2 <= 1500:
        return "#ff8a65"
    else:
        return "#ef5350"


def get_ch2o_color(ch2o: float) -> str:
    """甲醛按浓度返回颜色"""
    if ch2o <= 60:
        return "#66bb6a"
    elif ch2o <= 100:
        return "#ffb74d"
    else:
        return "#ef5350"


def get_tvoc_color(tvoc: float) -> str:
    """TVOC 按浓度返回颜色"""
    if tvoc <= 400:
        return "#66bb6a"
    elif tvoc <= 600:
        return "#ffb74d"
    elif tvoc <= 1000:
        return "#ff8a65"
    else:
        return "#ef5350"


# ==================== 数值卡片组件 ====================

class SensorCard(QFrame):
    """
    此类封装单个传感器数据的卡片式显示控件
    包含以下子功能:
        1.动态设置卡片标题、数值与单位
        2.支持通过 stylesheet 切换边框颜色，实现数值分级变色
    """

    def __init__(self, title: str, unit: str, color: str, parent=None):
        """
        功能: 初始化传感器卡片，设置标题、单位、默认边框颜色及布局
        输入: title - str，卡片标题文字（如 "温度"）
              unit  - str，数值单位（如 "°C"）
              color - str，CSS 颜色值，默认边框颜色
              parent - QWidget 或 None，父级控件
        输出: 无
        """
        super().__init__(parent)
        self._color = color
        self.setFrameShape(QFrame.StyledPanel)

        # 默认卡片样式
        self.setStyleSheet(f"""
            SensorCard {{
                background-color: #2d2d2d;
                border: 1px solid {color};
                border-radius: 10px;
            }}
        """)
        self.setMinimumSize(150, 110)

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)

        # 标题标签
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFont(QFont("Arial", 12))
        self.title_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        layout.addWidget(self.title_label)

        # 数值标签
        self.value_label = QLabel("--")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setFont(QFont("Courier New", 36, QFont.Bold))
        self.value_label.setStyleSheet(f"color: {color};")
        layout.addWidget(self.value_label)

        # 单位标签
        self.unit_label = QLabel(unit)
        self.unit_label.setAlignment(Qt.AlignCenter)
        self.unit_label.setFont(QFont("Arial", 10))
        self.unit_label.setStyleSheet("color: #888;")
        layout.addWidget(self.unit_label)

    def set_value(self, value_str: str, color: str = None):
        """
        功能: 更新卡片上显示的数值文本，可选地同时修改数值颜色
        输入: value_str - str，格式化后的数值字符串（如 "28.4"）
              color     - str 或 None，CSS 颜色值；None 时保持当前颜色
        输出: 无
        """
        self.value_label.setText(value_str)
        if color is not None:
            self.value_label.setStyleSheet(f"color: {color};")


# ==================== 地图画布组件 ====================

class MapCanvas(QWidget):
    """
    此类负责自绘地图画布，用于实时显示无人机位置与动态高亮方块
    包含以下子功能:
        1.接收外部坐标更新无人机位置并记录轨迹
        2.根据当前坐标计算所在的 1×1 整数网格方块并高亮
        3.自绘坐标网格、高亮方块、轨迹线与无人机标识
        4.在高亮方块发生变化时发射信号通知外部

    高亮规则:
        若坐标 (x, y) 中 x 处于相邻整数 A 与 B 之间、y 处于相邻整数 C 与 D 之间，
        则高亮方块为 (A,C)(A,D)(B,C)(B,D) 围成的区域
        其中 A = floor(x), B = A+1, C = floor(y), D = C+1
    """

    # 地图可视范围（世界坐标）
    VIEW_X_MIN, VIEW_X_MAX = -8.0, 8.0
    VIEW_Y_MIN, VIEW_Y_MAX = -8.0, 8.0

    # 历史轨迹保留长度
    TRAIL_LENGTH = 20

    # 高亮方块变化信号（携带最新的方块边界 A, B, C, D）
    square_changed = pyqtSignal(int, int, int, int)

    def __init__(self, parent=None):
        """
        功能: 初始化地图画布，设置最小尺寸、轨迹缓冲区及初始方块边界
        输入: parent - QWidget 或 None，父级控件
        输出: 无
        """
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 轨迹缓冲区（固定长度双端队列）
        self._trail = collections.deque(maxlen=self.TRAIL_LENGTH)

        # 当前无人机坐标
        self._drone_x = 0.0
        self._drone_y = 0.0

        # 高亮方块边界：A=floor(x), B=A+1, C=floor(y), D=C+1
        self._square_A = 0
        self._square_B = 1
        self._square_C = 0
        self._square_D = 1

        self.setMouseTracking(True)

    # ==================== 公共接口 ====================

    def update_position(self, x: float, y: float):
        """
        功能: 由外部定时调用，更新无人机坐标、计算所在网格方块并触发重绘；
              若方块边界发生变化则发射 square_changed 信号
        输入: x - float，无人机当前 X 坐标
              y - float，无人机当前 Y 坐标
        输出: 无
        """
        self._drone_x = x
        self._drone_y = y
        self._trail.append((x, y))

        # 计算新的方块边界
        new_A, new_B, new_C, new_D = self._compute_square_bounds(x, y)

        # 判断方块是否发生变化（tuple 一次性比较）
        if (new_A, new_B, new_C, new_D) != (self._square_A, self._square_B,
                                               self._square_C, self._square_D):
            self._square_A, self._square_B, self._square_C, self._square_D = (
                new_A, new_B, new_C, new_D
            )
            self.square_changed.emit(new_A, new_B, new_C, new_D)

        self.update()  # 触发 paintEvent 重绘

    # ==================== 方块边界计算 ====================

    @staticmethod
    def _compute_square_bounds(x: float, y: float) -> tuple:
        """
        功能: 根据当前坐标计算所在的 1×1 整数网格方块边界
              规则: A = floor(x), B = A + 1, C = floor(y), D = C + 1
        输入: x - float，X 坐标
              y - float，Y 坐标
        输出: tuple (A, B, C, D) — 四个整数，分别表示 X 和 Y 方向的方块下界与上界
        示例: (2.3, -1.7) → A=2, B=3, C=-2, D=-1
        """
        A = math.floor(x)
        B = A + 1
        C = math.floor(y)
        D = C + 1
        return A, B, C, D

    # ==================== 坐标变换 ====================

    def _world_to_widget(self, wx: float, wy: float) -> QPointF:
        """
        功能: 将世界坐标转换为控件像素坐标，Y 轴翻转以适配屏幕坐标系
              X/Y 方向使用统一缩放比，保证单位长度一致，地图居中显示
        输入: wx - float，世界 X 坐标
              wy - float，世界 Y 坐标
        输出: QPointF - 对应的控件像素坐标点
        """
        margin = 30
        world_w = self.VIEW_X_MAX - self.VIEW_X_MIN  # 世界 X 范围宽度
        world_h = self.VIEW_Y_MAX - self.VIEW_Y_MIN  # 世界 Y 范围高度

        avail_w = self.width() - 2 * margin
        avail_h = self.height() - 2 * margin

        # 取 X/Y 中较小的缩放比，保证两个方向上单位长度一致
        scale = min(avail_w / world_w, avail_h / world_h)

        draw_w = scale * world_w  # 实际绘制宽度
        draw_h = scale * world_h  # 实际绘制高度

        # 在可用区域内居中偏移
        offset_x = (avail_w - draw_w) / 2
        offset_y = (avail_h - draw_h) / 2

        # X 方向线性映射
        px = margin + offset_x + (wx - self.VIEW_X_MIN) * scale
        # Y 方向线性映射 + 翻转（屏幕 Y 轴朝下，世界 Y 轴朝上）
        py = margin + offset_y + (self.VIEW_Y_MAX - wy) * scale

        return QPointF(px, py)

    def _widget_to_world(self, px: float, py: float) -> tuple:
        """像素坐标 → 世界坐标（_world_to_widget 的逆变换）"""
        margin = 30
        world_w = self.VIEW_X_MAX - self.VIEW_X_MIN
        world_h = self.VIEW_Y_MAX - self.VIEW_Y_MIN
        avail_w = self.width() - 2 * margin
        avail_h = self.height() - 2 * margin
        scale = min(avail_w / world_w, avail_h / world_h)
        draw_w = scale * world_w
        draw_h = scale * world_h
        offset_x = (avail_w - draw_w) / 2
        offset_y = (avail_h - draw_h) / 2
        wx = self.VIEW_X_MIN + (px - margin - offset_x) / scale
        wy = self.VIEW_Y_MAX - (py - margin - offset_y) / scale
        return wx, wy

    # ==================== 绘制入口 ====================

    def paintEvent(self, event):
        """
        功能: Qt 重绘事件入口，按顺序绘制背景、网格、正方形、轨迹、无人机与图例
        输入: event - QPaintEvent，重绘事件对象
        输出: 无
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 深色背景填充
        painter.fillRect(self.rect(), QColor("#1a1a2e"))

        self._draw_grid(painter)
        self._draw_highlight_square(painter)
        self._draw_trail(painter)
        self._draw_drone(painter)
        self._draw_legend(painter)

    def _draw_grid(self, painter: QPainter):
        """
        功能: 绘制坐标网格线与坐标轴数字标注
        输入: painter - QPainter，绘图对象
        输出: 无
        """
        pen_grid = QPen(QColor("#2a2a4a"), 1, Qt.DotLine)
        pen_axis = QPen(QColor("#555"), 1.5)

        # 纵向网格线（含坐标轴粗线）
        for ix in range(int(self.VIEW_X_MIN), int(self.VIEW_X_MAX) + 1):
            painter.setPen(pen_axis if ix == 0 else pen_grid)
            p1 = self._world_to_widget(ix, self.VIEW_Y_MIN)
            p2 = self._world_to_widget(ix, self.VIEW_Y_MAX)
            painter.drawLine(p1, p2)

        # 横向网格线（含坐标轴粗线）
        for iy in range(int(self.VIEW_Y_MIN), int(self.VIEW_Y_MAX) + 1):
            painter.setPen(pen_axis if iy == 0 else pen_grid)
            p1 = self._world_to_widget(self.VIEW_X_MIN, iy)
            p2 = self._world_to_widget(self.VIEW_X_MAX, iy)
            painter.drawLine(p1, p2)

        # X 轴数字标注
        painter.setPen(QColor("#888"))
        painter.setFont(QFont("Arial", 9))
        for ix in range(int(self.VIEW_X_MIN), int(self.VIEW_X_MAX) + 1):
            pt = self._world_to_widget(ix, self.VIEW_Y_MIN)
            painter.drawText(QPointF(pt.x() - 6, pt.y() + 18), str(ix))

        # Y 轴数字标注
        for iy in range(int(self.VIEW_Y_MIN), int(self.VIEW_Y_MAX) + 1):
            pt = self._world_to_widget(self.VIEW_X_MIN, iy)
            painter.drawText(QPointF(pt.x() - 24, pt.y() + 4), str(iy))

    def _draw_highlight_square(self, painter: QPainter):
        """
        功能: 根据当前方块边界 A,B,C,D 绘制高亮方块
              方块由 (A,C)(A,D)(B,D)(B,C) 围成
        输入: painter - QPainter，绘图对象
        输出: 无
        """
        A, B, C, D = self._square_A, self._square_B, self._square_C, self._square_D

        # 四角顶点（按顺时针顺序）
        p0 = self._world_to_widget(A, C)  # 左下
        p1 = self._world_to_widget(B, C)  # 右下
        p2 = self._world_to_widget(B, D)  # 右上
        p3 = self._world_to_widget(A, D)  # 左上

        poly = QPolygonF([p0, p1, p2, p3])

        # 高亮状态：亮绿色半透明填充 + 加粗实线边框
        painter.setBrush(QBrush(QColor(0, 255, 100, 80)))
        painter.setPen(QPen(QColor("#00ff66"), 3, Qt.SolidLine))
        painter.drawPolygon(poly)

    def _draw_trail(self, painter: QPainter):
        """
        功能: 绘制最近 TRAIL_LENGTH 个历史位置的连线轨迹
        输入: painter - QPainter，绘图对象
        输出: 无
        """
        if len(self._trail) < 2:
            return

        path = QPainterPath()
        first = self._world_to_widget(*self._trail[0])
        path.moveTo(first)

        for wx, wy in list(self._trail)[1:]:
            path.lineTo(self._world_to_widget(wx, wy))

        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(100, 180, 255, 120), 1.5, Qt.SolidLine))
        painter.drawPath(path)

    def _draw_drone(self, painter: QPainter):
        """
        功能: 在当前位置绘制无人机标识——红色圆点、白色十字准星与坐标文字
        输入: painter - QPainter，绘图对象
        输出: 无
        """
        center = self._world_to_widget(self._drone_x, self._drone_y)
        r = 8

        # 红色光晕
        painter.setBrush(QBrush(QColor(255, 60, 60, 60)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center, r + 6, r + 6)

        # 主体红色圆点
        painter.setBrush(QBrush(QColor("#ff3333")))
        painter.setPen(QPen(QColor("#fff"), 2))
        painter.drawEllipse(center, r, r)

        # 白色十字准星
        painter.setPen(QPen(QColor("#fff"), 1))
        painter.drawLine(QPointF(center.x() - r - 4, center.y()),
                         QPointF(center.x() + r + 4, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - r - 4),
                         QPointF(center.x(), center.y() + r + 4))

        # 当前坐标文字
        painter.setFont(QFont("Courier New", 9, QFont.Bold))
        painter.setPen(QColor("#fff"))
        text = f"({self._drone_x:.2f}, {self._drone_y:.2f})"
        painter.drawText(QPointF(center.x() - 40, center.y() - 18), text)

    def _draw_legend(self, painter: QPainter):
        """
        功能: 在画布左下角绘制图例——高亮方块样式与轨迹线样式说明
        输入: painter - QPainter，绘图对象
        输出: 无
        """
        painter.setFont(QFont("Arial", 9))
        x0, y0 = 8, self.height() - 60

        # 高亮方块图例
        painter.setPen(QPen(QColor("#00ff66"), 2))
        painter.setBrush(QBrush(QColor(0, 255, 100, 40)))
        painter.drawRect(x0, y0, 14, 14)
        painter.setPen(QColor("#ccc"))
        painter.drawText(x0 + 20, y0 + 12,
                         f"高亮方块: ({self._square_A},{self._square_C})"
                         f"→({self._square_B},{self._square_D})")

        # 轨迹线图例
        painter.setPen(QPen(QColor(100, 180, 255), 1.5))
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(x0, y0 + 28, x0 + 14, y0 + 28)
        painter.setPen(QColor("#ccc"))
        painter.drawText(x0 + 20, y0 + 33, "历史轨迹 (最近20点)")


# ==================== 传感器地图画布组件 ====================

class SensorMapCanvas(MapCanvas):
    """通用传感器地图画布：区块染色记录 + 点击查询，适用于温度/湿度/PM2.5"""

    cell_clicked = pyqtSignal(int, int, int, int, float, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._cell_data = {}
        self._selected_key = None  # 当前被点击选中的区块 (A, C)
        self.setMouseTracking(True)

    def record(self, x: float, y: float, value: float, color: str):
        """在当前坐标所在网格区块记录传感器值与颜色"""
        A, B, C, D = self._compute_square_bounds(x, y)
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._cell_data[(A, C)] = (value, color, now_str)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#1a1a2e"))
        self._draw_grid(painter)
        self._draw_cells(painter)
        self._draw_selected_cell(painter)
        self._draw_highlight_square(painter)
        self._draw_legend(painter)

    def _draw_cells(self, painter: QPainter):
        for (A, C), (_val, color_hex, _ts) in self._cell_data.items():
            B, D = A + 1, C + 1
            p0 = self._world_to_widget(A, C)
            p1 = self._world_to_widget(B, C)
            p2 = self._world_to_widget(B, D)
            p3 = self._world_to_widget(A, D)
            poly = QPolygonF([p0, p1, p2, p3])
            base = QColor(color_hex)
            fill = QColor(base.red(), base.green(), base.blue(), 100)
            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(QColor(color_hex), 1, Qt.SolidLine))
            painter.drawPolygon(poly)

    def _draw_selected_cell(self, painter: QPainter):
        """高亮绘制用户点击选中的区块：加粗亮色边框 + 更亮的半透明填充"""
        if self._selected_key is None:
            return
        A, C = self._selected_key
        if (A, C) not in self._cell_data:
            # 选中的是无数据区块，也绘制一个虚线提示框
            B, D = A + 1, C + 1
            p0 = self._world_to_widget(A, C)
            p1 = self._world_to_widget(B, C)
            p2 = self._world_to_widget(B, D)
            p3 = self._world_to_widget(A, D)
            poly = QPolygonF([p0, p1, p2, p3])
            painter.setBrush(QBrush(QColor(0, 255, 255, 30)))
            painter.setPen(QPen(QColor("#00ffff"), 2, Qt.DashLine))
            painter.drawPolygon(poly)
            return
        _val, color_hex, _ts = self._cell_data[(A, C)]
        B, D = A + 1, C + 1
        p0 = self._world_to_widget(A, C)
        p1 = self._world_to_widget(B, C)
        p2 = self._world_to_widget(B, D)
        p3 = self._world_to_widget(A, D)
        poly = QPolygonF([p0, p1, p2, p3])
        # 选中的区块：亮青色加粗边框 + 更亮的半透明填充
        base = QColor(color_hex)
        fill = QColor(base.red(), base.green(), base.blue(), 160)
        painter.setBrush(QBrush(fill))
        painter.setPen(QPen(QColor("#00ffff"), 3, Qt.SolidLine))
        painter.drawPolygon(poly)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() != Qt.LeftButton:
            return
        wx, wy = self._widget_to_world(event.x(), event.y())
        A, B, C, D = self._compute_square_bounds(wx, wy)
        key = (A, C)
        # 记录选中的区块，触发高亮重绘
        self._selected_key = key
        self.update()
        if key in self._cell_data:
            val, _c, ts = self._cell_data[key]
            self.cell_clicked.emit(A, B, C, D, val, ts)
        else:
            self.cell_clicked.emit(A, B, C, D, float('nan'), "")


# ==================== 主窗口 ====================

class StdinDataSource(QObject):
    """从 stdin 管道读取远程传感器数据（格式: 温度,湿度,PM2.5,PM10,CO2,CH2O,TVOC,x,y），通过信号发送"""

    data_received = pyqtSignal(float, float, float, float, float, float, float, float, float)
    report_received = pyqtSignal(str)  # DeepSeek 环境报告
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._buffer = ""
        self._fd = sys.stdin.fileno()
        self._notifier = QSocketNotifier(self._fd, QSocketNotifier.Read, self)
        self._notifier.activated.connect(self._read_stdin)

    def _read_stdin(self):
        try:
            data = os.read(self._fd, 4096).decode("utf-8", errors="replace")
            if not data:
                self._notifier.setEnabled(False)
                return
            self._buffer += data
            while "\n" in self._buffer:
                line, self._buffer = self._buffer.split("\n", 1)
                self._parse_line(line.strip())
        except Exception as e:
            self.error_occurred.emit(str(e))

    def _parse_line(self, line: str):
        if not line:
            return
        # DeepSeek 报告行
        if line.startswith("DEEPSEEK_REPORT:"):
            report = line[len("DEEPSEEK_REPORT:"):].strip()
            if report:
                self.report_received.emit(report)
            return
        try:
            parts = line.split(",")
            if len(parts) != 9:
                return
            t, h, p25, p10, co2, ch2o, tvoc, x, y = map(float, parts)
            self.data_received.emit(t, h, p25, p10, co2, ch2o, tvoc, x, y)
        except ValueError:
            pass


# ==================== 主窗口 ====================

class DashboardWindow(QMainWindow):
    """
    此类为应用主窗口，负责组装所有 UI 组件并驱动数据刷新
    包含以下子功能:
        1.构建八个标签页布局（总览 + 七种传感器专属地图）
        2.通过 StdinDataSource 从 stdin 管道实时读取远程传感器数据
        3.根据数据值动态切换卡片与地图的视觉样式
        4.响应地图高亮方块变化信号并更新状态栏提示
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("环境传感器数据仪表盘")
        self.setMinimumSize(960, 720)

        # 高亮方块当前边界（用于状态栏显示）
        self._sq_A, self._sq_B = 0, 1
        self._sq_C, self._sq_D = 0, 1

        # DeepSeek 报告历史（FIFO，最多保留 3 条）
        self._report_history = collections.deque(maxlen=3)

        self._init_ui()

        # 数据源：从 stdin 管道读取远程传感器数据
        self._data_source = StdinDataSource(self)
        self._data_source.data_received.connect(self._on_data_received)
        self._data_source.report_received.connect(self._on_report_received)
        self._data_source.error_occurred.connect(self._on_data_error)

        # 仅用于更新时间标签的 1 秒定时器
        self._time_timer = QTimer(self)
        self._time_timer.timeout.connect(self._update_time)
        self._time_timer.start(1000)

        # 初始时间显示
        self._update_time()

    # ==================== UI 构建 ====================

    def _init_ui(self):
        """
        功能: 构建八个标签页 UI 布局
              ① 总览（时间 + 七张卡片 + 地图 + 状态栏）
              ②~⑧ 温度 / 湿度 / PM2.5 / PM10 / eCO₂ / eCH₂O / TVOC（独立地图大屏）
              页面切换按钮位于顶部
        输入: 无
        输出: 无
        """
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabPosition(QTabWidget.North)
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #3c3c3c; background-color: #1e1e1e; }
            QTabBar::tab {
                background-color: #2d2d2d; color: #aaa; padding: 8px 24px;
                border: 1px solid #3c3c3c; border-bottom: none;
                border-top-left-radius: 6px; border-top-right-radius: 6px;
            }
            QTabBar::tab:selected { background-color: #1e1e1e; color: #00d4ff; font-weight: bold; }
            QTabBar::tab:hover { background-color: #3c3c3c; }
        """)
        self.setCentralWidget(self.tab_widget)

        # ---- 构建四个标签页 ----
        self.tab_widget.addTab(self._build_dashboard_tab(), "总览")
        self.tab_widget.addTab(self._build_sensor_map_tab("温度", "°C", "#ff6b6b", get_temp_color, "temp"), "温度")
        self.tab_widget.addTab(self._build_sensor_map_tab("湿度", "%", "#4fc3f7", get_humi_color, "humi"), "湿度")
        self.tab_widget.addTab(self._build_sensor_map_tab("PM2.5", "μg/m³", "#ffb74d", get_pm25_color, "pm25"), "PM2.5")
        self.tab_widget.addTab(self._build_sensor_map_tab("PM10", "μg/m³", "#ff9800", get_pm10_color, "pm10"), "PM10")
        self.tab_widget.addTab(self._build_sensor_map_tab("eCO₂", "ppm", "#8bc34a", get_co2_color, "co2"), "eCO₂")
        self.tab_widget.addTab(self._build_sensor_map_tab("eCH₂O", "μg/m³", "#e91e63", get_ch2o_color, "ch2o"), "eCH₂O")
        self.tab_widget.addTab(self._build_sensor_map_tab("TVOC", "μg/m³", "#9c27b0", get_tvoc_color, "tvoc"), "TVOC")

        # ---- 底部状态栏 ----
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet("color: #888;")
        self.setStatusBar(self.status_bar)

    # ==================== 标签页 ①：总览 ====================

    def _build_dashboard_tab(self) -> QWidget:
        """
        功能: 构建总览标签页，包含时间、三张传感器卡片与地图画布
        输入: 无
        输出: QWidget - 总览标签页控件
        """
        central = QWidget()
        root = QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(12, 12, 12, 12)

        # ---- ① 当前时间显示 ----
        time_group = QGroupBox("当前时间")
        time_layout = QHBoxLayout(time_group)
        self.time_label = QLabel("---- --:--:--")
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setFont(QFont("Courier New", 26, QFont.Bold))
        self.time_label.setStyleSheet("color: #00d4ff;")
        time_layout.addWidget(self.time_label)
        root.addWidget(time_group)

        # ---- ② 七张传感器卡片（温度 / 湿度 / PM2.5 / PM10 / CO2 / 甲醛 / TVOC）----
        card_grid = QGridLayout()
        card_grid.setHorizontalSpacing(8)
        card_grid.setVerticalSpacing(8)

        self.card_temp = SensorCard("温度", "°C", "#ff6b6b")
        self.card_humi = SensorCard("湿度", "%", "#4fc3f7")
        self.card_pm25 = SensorCard("PM2.5", "μg/m³", "#ffb74d")
        self.card_pm10 = SensorCard("PM10", "μg/m³", "#ff9800")
        self.card_co2 = SensorCard("eCO₂", "ppm", "#8bc34a")
        self.card_ch2o = SensorCard("eCH₂O", "μg/m³", "#e91e63")
        self.card_tvoc = SensorCard("TVOC", "μg/m³", "#9c27b0")

        # 第一行：温度 / 湿度 / PM2.5 / PM10
        card_grid.addWidget(self.card_temp, 0, 0)
        card_grid.addWidget(self.card_humi, 0, 1)
        card_grid.addWidget(self.card_pm25, 0, 2)
        card_grid.addWidget(self.card_pm10, 0, 3)
        # 第二行：CO2 / 甲醛 / TVOC（居中，第 0-1-2 列，共 3 张）
        card_grid.addWidget(self.card_co2, 1, 0)
        card_grid.addWidget(self.card_ch2o, 1, 1)
        card_grid.addWidget(self.card_tvoc, 1, 2)
        root.addLayout(card_grid)

        # ---- ③ 无人机位置地图 + DeepSeek 报告 ----
        map_group = QGroupBox("无人机位置地图 — 高亮方块由坐标所在网格决定")
        map_row = QHBoxLayout(map_group)
        map_row.setSpacing(10)

        self.map_canvas = MapCanvas()
        self.map_canvas.square_changed.connect(self._on_square_changed)
        map_row.addWidget(self.map_canvas, 3)  # stretch=3，地图占 75% 宽度

        # DeepSeek 环境报告面板（保留最近 3 条，最新在上）
        report_panel = QGroupBox("DeepSeek 环境报告")
        report_layout = QVBoxLayout(report_panel)
        report_layout.setSpacing(6)

        self.report_labels = []
        for i in range(3):
            lbl = QLabel(f"#{i + 1} 等待环境报告...")
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            lbl.setWordWrap(True)
            lbl.setFont(QFont("Microsoft YaHei", 9))
            lbl.setStyleSheet("""
                color: #b0e0b0;
                background-color: #1a2a1a;
                border: 1px solid #3a5a3a;
                border-radius: 6px;
                padding: 8px;
            """)
            lbl.setMinimumHeight(60)
            lbl.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
            report_layout.addWidget(lbl)
            self.report_labels.append(lbl)

        map_row.addWidget(report_panel, 1)  # stretch=1，报告面板占 25% 宽度

        root.addWidget(map_group, 1)  # stretch=1 使地图区域撑满剩余空间

        return central

    # ==================== 标签页 ②③④：通用传感器地图标签页 ====================

    def _build_sensor_map_tab(self, title: str, unit: str, default_color: str,
                               color_func, key: str) -> QWidget:
        """构建传感器地图标签页——顶部双框（时间 | 数值）+ 地图画布"""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(6)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        time_box = QGroupBox("记录时间")
        time_box_layout = QVBoxLayout(time_box)
        time_label = QLabel("无记录")
        time_label.setAlignment(Qt.AlignCenter)
        time_label.setFont(QFont("Courier New", 16, QFont.Bold))
        time_label.setStyleSheet("color: #00d4ff;")
        time_label.setMinimumHeight(50)
        time_box_layout.addWidget(time_label)
        top_row.addWidget(time_box)

        value_box = QGroupBox(title)
        value_box_layout = QVBoxLayout(value_box)
        value_label = QLabel("无记录")
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setFont(QFont("Courier New", 16, QFont.Bold))
        value_label.setStyleSheet(f"color: {default_color};")
        value_label.setMinimumHeight(50)
        value_box_layout.addWidget(value_label)
        top_row.addWidget(value_box)

        coord_box = QGroupBox("区块坐标")
        coord_box_layout = QVBoxLayout(coord_box)
        coord_label = QLabel("点击地图区块查询")
        coord_label.setAlignment(Qt.AlignCenter)
        coord_label.setFont(QFont("Courier New", 13, QFont.Bold))
        coord_label.setStyleSheet("color: #ffd740;")
        coord_label.setMinimumHeight(50)
        coord_label.setWordWrap(True)
        coord_box_layout.addWidget(coord_label)
        top_row.addWidget(coord_box)

        layout.addLayout(top_row)

        sensor_map = SensorMapCanvas()
        sensor_map.cell_clicked.connect(
            lambda A, B, C, D, val, ts, tl=time_label, vl=value_label, cl=coord_label, u=unit, cf=color_func:
                self._on_sensor_cell_clicked(tl, vl, cl, u, cf, A, B, C, D, val, ts))
        layout.addWidget(sensor_map, 1)

        setattr(self, f"{key}_time_label", time_label)
        setattr(self, f"{key}_value_label", value_label)
        setattr(self, f"{key}_coord_label", coord_label)
        setattr(self, f"{key}_map", sensor_map)
        return page

    # ==================== 信号槽 ====================

    def _on_square_changed(self, A: int, B: int, C: int, D: int):
        self._sq_A, self._sq_B = A, B
        self._sq_C, self._sq_D = C, D
        self.status_bar.showMessage(
            f"高亮方块切换至: ({A},{C})→({B},{D})", 2000)

    def _on_sensor_cell_clicked(self, time_label, value_label, coord_label, unit_str, color_func,
                                  A: int, B: int, C: int, D: int, val: float, ts: str):
        # 更新坐标显示
        coord_label.setText(f"({A},{C}) → ({B},{D})")
        coord_label.setStyleSheet("color: #ffd740;")
        if math.isnan(val):
            time_label.setText("无记录")
            time_label.setStyleSheet("color: #888;")
            value_label.setText("无记录")
            value_label.setStyleSheet("color: #888;")
        else:
            time_label.setText(ts)
            time_label.setStyleSheet("color: #00d4ff;")
            value_label.setText(f"{val:.1f} {unit_str}")
            value_label.setStyleSheet(f"color: {color_func(val)};")

    # ==================== 数据刷新 ====================

    def _on_data_received(self, t: float, h: float, p25: float, p10: float,
                          co2: float, ch2o: float, tvoc: float, x: float, y: float):
        """stdin 管道数据到达时触发，驱动全量 UI 刷新"""
        now = datetime.now()

        self.time_label.setText(now.strftime("%Y-%m-%d  %H:%M:%S"))

        # 温度动态变色
        temp_color = get_temp_color(t)
        self.card_temp.set_value(f"{t:.1f}", temp_color)
        self.temp_map.record(x, y, t, temp_color)
        self.temp_map.update_position(x, y)

        # 湿度
        humi_color = get_humi_color(h)
        self.card_humi.set_value(f"{h:.1f}", humi_color)
        self.humi_map.record(x, y, h, humi_color)
        self.humi_map.update_position(x, y)

        # PM2.5
        pm25_color = get_pm25_color(p25)
        if p25 <= 50:
            pm25_text = "优"
        elif p25 <= 100:
            pm25_text = "良"
        elif p25 <= 150:
            pm25_text = "轻度"
        elif p25 <= 200:
            pm25_text = "中度"
        else:
            pm25_text = "重度"
        self.card_pm25.set_value(f"{p25:.0f}", pm25_color)
        self.pm25_map.record(x, y, p25, pm25_color)
        self.pm25_map.update_position(x, y)

        # PM10
        pm10_color = get_pm10_color(p10)
        self.card_pm10.set_value(f"{p10:.0f}", pm10_color)
        self.pm10_map.record(x, y, p10, pm10_color)
        self.pm10_map.update_position(x, y)

        # eCO2
        co2_color = get_co2_color(co2)
        self.card_co2.set_value(f"{co2:.0f}", co2_color)
        self.co2_map.record(x, y, co2, co2_color)
        self.co2_map.update_position(x, y)

        # eCH2O
        ch2o_color = get_ch2o_color(ch2o)
        self.card_ch2o.set_value(f"{ch2o:.0f}", ch2o_color)
        self.ch2o_map.record(x, y, ch2o, ch2o_color)
        self.ch2o_map.update_position(x, y)

        # TVOC
        tvoc_color = get_tvoc_color(tvoc)
        self.card_tvoc.set_value(f"{tvoc:.0f}", tvoc_color)
        self.tvoc_map.record(x, y, tvoc, tvoc_color)
        self.tvoc_map.update_position(x, y)

        # 总览地图
        self.map_canvas.update_position(x, y)

        # 状态栏
        self.status_bar.showMessage(
            f"更新于 {now.strftime('%H:%M:%S')}  |  "
            f"温度 {t:.1f}°C  |  湿度 {h:.1f}%  |  "
            f"PM2.5 {p25:.0f} ({pm25_text})  |  PM10 {p10:.0f}  |  "
            f"CO₂ {co2:.0f}  |  CH₂O {ch2o:.0f}  |  TVOC {tvoc:.0f}  |  "
            f"X={x:.2f}  Y={y:.2f}  |  "
            f"高亮方块: ({self._sq_A},{self._sq_C})→({self._sq_B},{self._sq_D})"
        )

    def _on_data_error(self, err_msg: str):
        """stdin 读取异常时更新状态栏提示"""
        self.status_bar.showMessage(f"数据读取错误: {err_msg}", 5000)

    def _on_report_received(self, report: str):
        """DeepSeek 环境报告到达时更新右侧面板（FIFO，保留最近 3 条）"""
        self._report_history.append(report)
        # 刷新全部 3 个标签（#1 最新，在顶部）
        for i in range(3):
            if i < len(self._report_history):
                # 从最新到最旧：history[-1]→#1, history[-2]→#2, history[-3]→#3
                self.report_labels[i].setText(
                    f"#{i + 1}  {self._report_history[-(i + 1)]}"
                )
            else:
                self.report_labels[i].setText(f"#{i + 1} 等待环境报告...")

    def _update_time(self):
        """仅更新总览时间标签（每秒触发）"""
        self.time_label.setText(datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))

# ==================== 全局暗色主题 ====================

DARK_STYLESHEET = """
QMainWindow { background-color: #1e1e1e; }
QGroupBox {
    font-weight: bold; color: #cccccc;
    border: 1px solid #3c3c3c; border-radius: 6px;
    margin-top: 12px; padding-top: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 12px; padding: 0 6px;
}
QStatusBar { background-color: #252525; color: #888; }
"""


# ==================== 入口 ====================

def main():
    """
    功能: 应用入口函数，创建 QApplication、应用暗色主题、启动主窗口并进入事件循环
    输入: 无（由命令行参数自动传入）
    输出: int - 应用退出码
    """
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLESHEET)

    window = DashboardWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
