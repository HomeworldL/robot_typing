import sys
import mujoco
import numpy as np
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QTextEdit,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSplitter,
)
from PySide6.QtCore import Qt, QThread, Signal, QMutex
from PySide6.QtGui import QPainter, QColor, QLinearGradient, QFont, QFontMetrics

# from mujoco_viewer import MujocoViewer
import mujoco.viewer
from pathlib import Path
import json
import time


class KeyHeatmap(QWidget):
    def __init__(self, key_layout):
        super().__init__()
        self.key_data = key_layout  # 从JSON加载的布局数据
        self.force_values = {}  # 传感器名称: 力值
        self.max_force = 10  # 对应传感器cutoff值
        self.scale = 5000  # 米到像素的转换比例
        self.setMinimumSize(1500, 500)

        # 配置字体
        self.font = QFont("Arial", 10)
        self.font.setBold(True)
        self.metrics = QFontMetrics(self.font)

        # 初始化颜色梯度
        self.gradient = QLinearGradient(0, 0, 1, 0)
        self.gradient.setColorAt(0.0, QColor(30, 144, 255))  # DodgerBlue
        self.gradient.setColorAt(1.0, QColor(255, 215, 0))  # Gold

    def update_forces(self, forces):
        """更新力值数据"""
        self.force_values = forces
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setFont(self.font)

        x_offset = self.key_data["1"]["position"][1]
        y_offset = self.key_data["1"]["position"][0]

        # 绘制所有按键
        for key in self.key_data.values():
            self._draw_key(painter, key, x_offset, y_offset)

    def _draw_key(self, painter, key, x_offset, y_offset):
        """绘制单个按键元素"""
        # 转换坐标到像素
        x = self._to_px(-key["position"][1] + x_offset)
        y = self._to_px(-key["position"][0] + y_offset)
        w = self._to_px(key["size"][0]) * 1.5
        h = self._to_px(key["size"][1]) * 1.5

        if w == 0 or h == 0:
            return

        # 获取当前力值
        force = self.force_values.get(key["sensor_name"], 0)
        ratio = min(force / self.max_force, 1.0)

        # 绘制背景
        color = self._get_color(ratio)
        painter.setBrush(color)
        painter.drawRoundedRect(x, y, w, h, 5, 5)

        # 绘制文本（两行显示）
        self._draw_key_label(painter, x, y, w, h, key["name"], force)

    def _to_px(self, meters):
        """米转像素"""
        return int(meters * self.scale)

    def _get_color(self, ratio):
        """根据力值比例获取颜色"""
        r = int(30 + 225 * ratio)
        g = int(144 + 71 * ratio)
        b = int(255 - 255 * ratio)
        return QColor(r, g, b)

    def _draw_key_label(self, painter, x, y, w, h, label, force):
        """绘制按键标签和力值（两行）"""
        # 第一行文本：按键标签
        text_rect = painter.boundingRect(x, y, w, h, Qt.AlignCenter, label)
        painter.setPen(Qt.black)
        painter.drawText(x + (w - text_rect.width()) / 2, y + h // 3, label)

        # 第二行文本：力值
        force_text = f"{force:.1f}N"
        force_rect = painter.boundingRect(x, y, w, h, Qt.AlignCenter, force_text)
        painter.setPen(Qt.darkGray)
        painter.drawText(x + (w - force_rect.width()) / 2, y + 2 * h // 3, force_text)


class SimulationThread(QThread):
    update_signal = Signal(dict)
    text_signal = Signal(str)

    def __init__(self, model, data):
        super().__init__()
        self.model = model
        self.data = data
        self.running = True

        # 新增状态管理变量
        self.last_trigger_time = {}  # 按键最后触发时间 {key: timestamp}
        self.current_key = None  # 当前激活的按键
        self.debounce_interval = 0.025  # 40Hz对应25ms周期
        self.key_press_start = {}  # 按键开始时间

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while self.running:
                # Measure the time it takes to step the simulation
                start_time = time.perf_counter()

                mujoco.mj_step(self.model, self.data)
                forces = self._collect_forces()
                self._detect_keypress(forces)

                # 处理持续按键
                if self.current_key:
                    elapsed = time.perf_counter() - self.key_press_start.get(
                        self.current_key, 0
                    )
                    if elapsed >= self.debounce_interval:
                        self._emit_key(self.current_key)
                        self.key_press_start[self.current_key] = time.perf_counter()
                self.update_signal.emit(forces)

                viewer.sync()
                step_time = time.perf_counter() - start_time
                # Calculate the time to sleep until the next time step
                time_until_next_step = self.model.opt.timestep - step_time
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
            self.running = False

    def _collect_forces(self):
        forces = {}
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            if name.startswith("touch_"):
                forces[name] = self.data.sensordata[i]
        return forces

    def _detect_keypress(self, forces, threshold=1):
        """改进的按键检测逻辑"""
        active_keys = []

        # 第一步：检测所有激活的按键
        for sensor, value in forces.items():
            if value > threshold:
                key = self._sensor_to_char(sensor)
                active_keys.append((sensor, key, time.perf_counter()))

        # 第二步：处理按键优先级
        if active_keys:
            # 选择最新激活的按键（最后触发的）
            newest = max(active_keys, key=lambda x: x)
            new_sensor, new_key, timestamp = newest

            # 如果当前没有激活按键，或者检测到新按键
            if not self.current_key or new_key != self.current_key:
                # 更新当前激活按键
                self.current_key = new_key
                self._emit_key(new_key)
                self.key_press_start[new_key] = timestamp
        else:
            # 没有激活按键时重置状态
            self.current_key = None
            self.key_press_start.clear()

    def _emit_key(self, key):
        """带防抖的按键发射"""
        now = time.perf_counter()
        last_time = self.last_trigger_time.get(key, 0)

        if now - last_time >= self.debounce_interval:
            self.text_signal.emit(key)
            self.last_trigger_time[key] = now

    def _sensor_to_char(self, sensor):
        # 转换sensor名称到字符
        key = sensor.replace("touch_", "")
        # special_map = {"space": " ", "backspace": "\b", "enter1": "\n", "enter2": "\n"}
        special_map = {
            "space": "space",
            "backspace": "backspace",
            "enter1": "enter1",
            "enter2": "enter2",
        }
        return special_map.get(key, key.upper())


class MainWindow(QMainWindow):
    def __init__(self, model_path):
        super().__init__()
        self.setWindowTitle("view_keyboard_touch")
        self.showMaximized()
        self.setMinimumSize(1500, 800)

        # 读取键盘布局
        self.model_path = model_path
        self.model_dir = Path(model_path).parent
        self.coords_path = self.model_dir / "keyboard_coords.json"
        self.key_layout = json.load(open(self.coords_path))
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        # self.viewer = MujocoViewer(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)  # 重置数据

        # 主界面布局
        main_widget = QWidget()
        layout = QVBoxLayout()

        # 使用 QSplitter 实现可拖动布局
        splitter = QSplitter(Qt.Vertical)  # 垂直分割器

        # 热力图
        self.heatmap = KeyHeatmap(self.key_layout)
        splitter.addWidget(self.heatmap)

        # 文本输出框
        self.text_output = QTextEdit()
        self.text_output.setReadOnly(True)
        self.text_output.setLineWrapMode(QTextEdit.WidgetWidth)  # 自动换行
        self.text_output.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)  # 显示滚动条

        # 将文本输出框包装到 QWidget 中，以便添加标签
        output_widget = QWidget()
        output_layout = QVBoxLayout()
        output_layout.addWidget(QLabel("output:"))
        output_layout.addWidget(self.text_output)
        output_widget.setLayout(output_layout)

        # 添加文本输出框到分割器
        splitter.addWidget(output_widget)

        # 设置初始比例（热力图占 2/3，文本输出框占 1/3）
        # splitter.setSizes([int(self.height() * 2 / 3), int(self.height() * 1 / 3)])

        # 将分割器添加到主布局
        layout.addWidget(splitter)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        # 启动仿真线程
        self.sim_thread = SimulationThread(self.model, self.data)
        self.sim_thread.update_signal.connect(self.heatmap.update_forces)
        self.sim_thread.text_signal.connect(self.update_text)
        self.sim_thread.start()

    def update_text(self, char):
        cursor = self.text_output.textCursor()
        if char == "\b":  # 退格处理
            cursor.deletePreviousChar()
        else:
            cursor.insertText(char)
        self.text_output.setTextCursor(cursor)
        self.text_output.ensureCursorVisible()

    def closeEvent(self, event):
        self.sim_thread.running = False
        self.sim_thread.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow("assets/keyboards/keyboard_1/scene.xml")
    window.show()
    sys.exit(app.exec())
