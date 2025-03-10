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
)
from PySide6.QtCore import Qt, QThread, Signal, QMutex
from PySide6.QtGui import QPainter, QColor, QLinearGradient
from mujoco_viewer import MujocoViewer
from pathlib import Path
import json


class KeyHeatmap(QWidget):
    def __init__(self, key_layout):
        super().__init__()
        self.key_layout = key_layout  # 预定义的键盘布局数据
        self.force_data = {}
        self.max_force = 1000  # 对应传感器的cutoff值
        self.setMinimumSize(800, 300)

        # 热力颜色梯度
        self.gradient = QLinearGradient(0, 0, 1, 0)
        self.gradient.setColorAt(0.0, QColor(0, 0, 255))  # 蓝色
        self.gradient.setColorAt(1.0, QColor(255, 255, 0))  # 黄色

    def update_forces(self, forces):
        self.force_data = forces
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 绘制热力图
        for row in self.key_layout:
            y_pos = row["position"]
            for key in row["keys"]:
                # 计算键位位置
                x, y = key["position"]
                w, h = key["size"]

                # 获取当前力值
                force = self.force_data.get(key["sensor_name"], 0)
                ratio = min(force / self.max_force, 1.0)

                # 绘制背景色
                color = self.gradientStopsAt(ratio)
                painter.setBrush(color)
                painter.drawRect(x, y, w, h)

                # 绘制文字
                painter.setPen(Qt.black)
                text = f"{key['label']}\n{force:.0f}N"
                painter.drawText(x + 5, y + 15, text)

    def gradientStopsAt(self, ratio):
        c1 = QColor(0, 0, 255)
        c2 = QColor(255, 255, 0)
        return QColor(
            c1.red() + (c2.red() - c1.red()) * ratio,
            c1.green() + (c2.green() - c1.green()) * ratio,
            c1.blue() + (c2.blue() - c1.blue()) * ratio,
        )


class SimulationThread(QThread):
    update_signal = Signal(dict)
    text_signal = Signal(str)

    def __init__(self, model_path):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = MujocoViewer(self.model, self.data)
        self.running = True
        self.mutex = QMutex()

    def run(self):
        while self.running:
            self.mutex.lock()
            mujoco.mj_step(self.model, self.data)
            forces = self._collect_forces()
            self._detect_keypress(forces)
            self.update_signal.emit(forces)
            self.mutex.unlock()
            self.viewer.render()

    def _collect_forces(self):
        forces = {}
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            if name.startswith("touch_"):
                forces[name] = self.data.sensordata[i]
        return forces

    def _detect_keypress(self, forces, threshold=500):
        for sensor, force in forces.items():
            if force > threshold:
                char = self._sensor_to_char(sensor)
                self.text_signal.emit(char)

    def _sensor_to_char(self, sensor):
        # 转换sensor名称到字符
        key = sensor.replace("touch_", "")
        special_map = {"space": " ", "backspace": "\b", "enter1": "\n", "enter2": "\n"}
        return special_map.get(key, key.upper())


class MainWindow(QMainWindow):
    def __init__(self, model_path):
        super().__init__()
        self.setWindowTitle("view_keyboard_touch")

        # 读取键盘布局
        self.model_path = model_path
        self.model_dir = Path(model_path).parent
        self.coords_path = self.model_dir / "keyboard_coords.json"
        self.key_layout = json.load(open(self.coords_path))

        # 界面布局
        main_widget = QWidget()
        layout = QVBoxLayout()

        # 热力图
        self.heatmap = KeyHeatmap(self.key_layout)
        layout.addWidget(self.heatmap)

        # 文本输出框
        self.text_output = QTextEdit()
        self.text_output.setReadOnly(True)
        layout.addWidget(QLabel("output:"))
        layout.addWidget(self.text_output)

        # 启动仿真线程
        # self.sim_thread = SimulationThread(model_path)
        # self.sim_thread.update_signal.connect(self.heatmap.update_forces)
        # self.sim_thread.text_signal.connect(self.update_text)
        # self.sim_thread.start()

        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

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
    window = MainWindow("assets/keyboards/keyboard_1/keyboard_1_processed.xml")
    window.show()
    sys.exit(app.exec())
