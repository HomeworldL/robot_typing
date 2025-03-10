import sys
import json
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.dom import minidom
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QFileDialog,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QMessageBox,
)


class KeyboardConfigurator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MuJoCo Keyboard Configurator")
        self.setMinimumSize(600, 400)

        # 初始化参数
        self.params = {
            "base": {
                "vertical_spacing": 0.018,
                "horizontal_spacing": 0.018,
                "key_width": 0.01,
                "key_height": 0.01,
                "A_y": 0.005,
                "Q_y": 0.01,
                "1_y": 0.015,
            },
            "special": {
                "space_width": 0.12,
                "backspace_y": -0.25,
                "backspace_width": 0.035,
                "enter_width1": 0.02,
                "enter_height1": 0.03,
                "enter_x1": 0.06,
                "enter_y1": -0.18,
                "enter_width2": 0.0,
                "enter_height2": 0.0,
                "enter_x2": 0.0,
                "enter_y2": 0.0,
                "camera_height": 0.3,
            },
        }

        # 初始化UI
        self.init_ui()
        self.source_xml_path = None

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        layout = QVBoxLayout()
        main_widget.setLayout(layout)

        # 文件选择
        file_layout = QHBoxLayout()
        self.source_file_btn = QPushButton("选择源XML")
        self.source_file_btn.clicked.connect(self.select_source_xml)
        self.source_file_label = QLabel("未选择文件")
        file_layout.addWidget(self.source_file_btn)
        file_layout.addWidget(self.source_file_label)

        # 参数标签页
        tabs = QTabWidget()
        self.base_tab = self.create_base_tab()
        self.special_tab = self.create_special_tab()
        tabs.addTab(self.base_tab, "基础参数")
        tabs.addTab(self.special_tab, "特殊按键")

        # 操作按钮
        btn_layout = QHBoxLayout()
        self.preview_btn = QPushButton("预览模型")
        self.preview_btn.clicked.connect(self.preview_model)
        self.save_final_btn = QPushButton("保存最终XML")
        self.save_final_btn.clicked.connect(self.save_final_xml)
        self.save_btn = QPushButton("保存配置")
        self.save_btn.clicked.connect(self.save_config)
        self.load_btn = QPushButton("加载配置")
        self.load_btn.clicked.connect(self.load_config)

        btn_layout.addWidget(self.preview_btn)
        btn_layout.addWidget(self.save_final_btn)
        btn_layout.addWidget(self.save_btn)
        btn_layout.addWidget(self.load_btn)

        # 组装界面
        layout.addLayout(file_layout)
        layout.addWidget(tabs)
        layout.addLayout(btn_layout)

    def create_base_tab(self):
        tab = QWidget()
        layout = QGridLayout()

        # 基础参数输入
        row = 0
        layout.addWidget(QLabel("垂直间距 (m):"), row, 0)
        self.vertical_spacing = QLineEdit(str(self.params["base"]["vertical_spacing"]))
        layout.addWidget(self.vertical_spacing, row, 1)

        row += 1
        layout.addWidget(QLabel("水平间距 (m):"), row, 0)
        self.horizontal_spacing = QLineEdit(
            str(self.params["base"]["horizontal_spacing"])
        )
        layout.addWidget(self.horizontal_spacing, row, 1)

        row += 1
        layout.addWidget(QLabel("普通按键宽度 (m):"), row, 0)
        self.key_width = QLineEdit(str(self.params["base"]["key_width"]))
        layout.addWidget(self.key_width, row, 1)

        row += 1
        layout.addWidget(QLabel("普通按键高度 (m):"), row, 0)
        self.key_height = QLineEdit(str(self.params["base"]["key_height"]))
        layout.addWidget(self.key_height, row, 1)

        row += 1
        layout.addWidget(QLabel("A键Y坐标 (m):"), row, 0)
        self.A_y = QLineEdit(str(self.params["base"]["A_y"]))
        layout.addWidget(self.A_y, row, 1)

        row += 1
        layout.addWidget(QLabel("Q键Y坐标 (m):"), row, 0)
        self.Q_y = QLineEdit(str(self.params["base"]["Q_y"]))
        layout.addWidget(self.Q_y, row, 1)

        row += 1
        layout.addWidget(QLabel("1键Y坐标 (m):"), row, 0)
        self.num1_y = QLineEdit(str(self.params["base"]["1_y"]))
        layout.addWidget(self.num1_y, row, 1)

        tab.setLayout(layout)
        return tab

    def create_special_tab(self):
        tab = QWidget()
        layout = QGridLayout()

        special = self.params["special"]

        # 空格参数
        row = 0
        layout.addWidget(QLabel("空格宽度 (m):"), row, 0)
        self.space_width = QLineEdit(str(special["space_width"]))
        layout.addWidget(self.space_width, row, 1)

        # 退格参数
        row += 1
        layout.addWidget(QLabel("退格Y偏移 (m):"), row, 0)
        self.backspace_y = QLineEdit(str(special["backspace_y"]))
        layout.addWidget(self.backspace_y, row, 1)

        row += 1
        layout.addWidget(QLabel("退格宽度 (m):"), row, 0)
        self.backspace_width = QLineEdit(str(special["backspace_width"]))
        layout.addWidget(self.backspace_width, row, 1)

        # 回车参数 - 部件1
        row += 1
        layout.addWidget(QLabel("回车部件1宽度 (m):"), row, 0)
        self.enter_w1 = QLineEdit(str(special["enter_width1"]))
        layout.addWidget(self.enter_w1, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件1高度 (m):"), row, 0)
        self.enter_h1 = QLineEdit(str(special["enter_height1"]))
        layout.addWidget(self.enter_h1, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件1X (m):"), row, 0)
        self.enter_x1 = QLineEdit(str(special["enter_x1"]))
        layout.addWidget(self.enter_x1, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件1Y (m):"), row, 0)
        self.enter_y1 = QLineEdit(str(special["enter_y1"]))
        layout.addWidget(self.enter_y1, row, 1)

        # 回车参数 - 部件2
        row += 1
        layout.addWidget(QLabel("回车部件2宽度 (m):"), row, 0)
        self.enter_w2 = QLineEdit(str(special["enter_width2"]))
        layout.addWidget(self.enter_w2, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件2高度 (m):"), row, 0)
        self.enter_h2 = QLineEdit(str(special["enter_height2"]))
        layout.addWidget(self.enter_h2, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件2X (m):"), row, 0)
        self.enter_x2 = QLineEdit(str(special["enter_x2"]))
        layout.addWidget(self.enter_x2, row, 1)

        row += 1
        layout.addWidget(QLabel("回车部件2Y (m):"), row, 0)
        self.enter_y2 = QLineEdit(str(special["enter_y2"]))
        layout.addWidget(self.enter_y2, row, 1)

        row += 1
        layout.addWidget(QLabel("相机高度 (m):"), row, 0)
        self.camera_height = QLineEdit(str(special["camera_height"]))
        layout.addWidget(self.camera_height, row, 1)

        tab.setLayout(layout)
        return tab

    def update_params_from_ui(self):
        """从UI更新参数"""
        try:
            # 基础参数
            base = self.params["base"]
            base["vertical_spacing"] = float(self.vertical_spacing.text())
            base["horizontal_spacing"] = float(self.horizontal_spacing.text())
            base["key_width"] = float(self.key_width.text())
            base["key_height"] = float(self.key_height.text())
            base["A_y"] = float(self.A_y.text())
            base["Q_y"] = float(self.Q_y.text())
            base["1_y"] = float(self.num1_y.text())

            # 特殊参数
            special = self.params["special"]
            special["space_width"] = float(self.space_width.text())
            special["backspace_y"] = float(self.backspace_y.text())
            special["backspace_width"] = float(self.backspace_width.text())
            special["enter_width1"] = float(self.enter_w1.text())
            special["enter_height1"] = float(self.enter_h1.text())
            special["enter_x1"] = float(self.enter_x1.text())
            special["enter_y1"] = float(self.enter_y1.text())
            special["enter_width2"] = float(self.enter_w2.text())
            special["enter_height2"] = float(self.enter_h2.text())
            special["enter_x2"] = float(self.enter_x2.text())
            special["enter_y2"] = float(self.enter_y2.text())
            special["camera_height"] = float(self.camera_height.text())

        except ValueError as e:
            print(f"参数格式错误: {e}")
            QMessageBox.warning(self, "输入错误", "请输入有效的数值参数")

    def validate_inputs(self):
        """参数范围验证"""
        errors = []
        if self.params["base"]["key_width"] <= 0:
            errors.append("按键宽度必须大于0")
        if self.params["special"]["enter_height1"] <= 0:
            errors.append("回车高度必须大于0")
        # 其他验证...

        if errors:
            QMessageBox.warning(self, "验证错误", "\n".join(errors))
            return False
        return True

    def select_source_xml(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "选择源XML文件", "", "XML Files (*.xml)"
        )
        if path:
            self.source_xml_path = path
            self.source_file_label.setText(Path(path).name)

    def preview_model(self):
        """生成临时XML并预览"""
        if not self.source_xml_path:
            return

        self.update_params_from_ui()

        # 生成临时文件
        temp_dir = Path(self.source_xml_path).parent
        xml_stem = Path(self.source_xml_path).stem
        temp_xml = temp_dir / f"{xml_stem}_preview.xml"
        # 生成模型
        generate_keyboard_model(self.source_xml_path, str(temp_xml), self.params)
        print(f"临时XML文件已生成：{temp_xml}")

        # # 启动查看器
        # self.run_mujoco_viewer(temp_xml)

    def save_final_xml(self):
        if not self.source_xml_path:
            return

        self.update_params_from_ui()

        temp_dir = Path(self.source_xml_path).parent
        xml_stem = Path(self.source_xml_path).stem
        temp_xml = temp_dir / f"{xml_stem}_processed.xml"
        # 生成模型
        generate_keyboard_model(self.source_xml_path, str(temp_xml), self.params, final=True)
        print(f"最终XML文件已生成：{temp_xml}")

    def run_mujoco_viewer(self, xml_path):
        """运行Mujoco查看器"""
        cmd = f"python -m mujoco.viewer --mjcf {xml_path}"
        try:
            subprocess.Popen(cmd, shell=True)
        except Exception as e:
            print(f"启动查看器失败: {e}")

    def save_config(self):
        """保存配置到文件，默认路径为源XML文件所在目录下的 'keyboard_config.json'"""
        if not self.source_xml_path:
            QMessageBox.warning(self, "未选择源文件", "请先选择源XML文件")
            return

        # 设置默认保存路径
        default_dir = Path(self.source_xml_path).parent
        default_path = str(default_dir / "keyboard_config.json")

        # 打开文件保存对话框
        path, _ = QFileDialog.getSaveFileName(
            self, "保存配置", default_path, "JSON Files (*.json)"  # 默认路径和文件名
        )

        # 如果用户选择了路径（未取消对话框）
        if path:
            # 确保文件扩展名为 .json
            if not path.lower().endswith(".json"):
                path += ".json"

            # 保存参数到文件
            try:
                with open(path, "w") as f:
                    json.dump(self.params, f, indent=2)
                QMessageBox.information(self, "保存成功", f"配置已保存到: {path}")
            except Exception as e:
                QMessageBox.critical(self, "保存失败", f"保存配置时出错: {e}")

    def load_config(self):
        """从文件加载配置，更新界面显示"""
        # 设置默认加载路径为源XML文件所在目录
        if self.source_xml_path:
            default_dir = str(Path(self.source_xml_path).parent)
        else:
            default_dir = ""

        # 打开文件选择对话框
        path, _ = QFileDialog.getOpenFileName(
            self, "加载配置", default_dir, "JSON Files (*.json)"  # 默认路径
        )

        # 如果用户选择了文件
        if path:
            try:
                # 加载配置文件
                with open(path, "r") as f:
                    self.params = json.load(f)

                # 更新界面显示
                self.update_ui()

                # 提示加载成功
                QMessageBox.information(self, "加载成功", f"配置已从 {path} 加载")
            except Exception as e:
                # 处理加载失败
                QMessageBox.critical(self, "加载失败", f"加载配置时出错: {e}")

    def update_ui(self):
        """用当前参数更新UI"""
        if not self.params:
            return

        # 基础参数
        base = self.params.get("base", {})
        self.vertical_spacing.setText(str(base.get("vertical_spacing", "")))
        self.horizontal_spacing.setText(str(base.get("horizontal_spacing", "")))
        self.key_width.setText(str(base.get("key_width", "")))
        self.key_height.setText(str(base.get("key_height", "")))
        self.A_y.setText(str(base.get("A_y", "")))
        self.Q_y.setText(str(base.get("Q_y", "")))
        self.num1_y.setText(str(base.get("1_y", "")))

        # 特殊参数
        special = self.params.get("special", {})
        self.space_width.setText(str(special.get("space_width", "")))
        self.backspace_y.setText(str(special.get("backspace_y", "")))
        self.backspace_width.setText(str(special.get("backspace_width", "")))
        self.enter_w1.setText(str(special.get("enter_width1", "")))
        self.enter_h1.setText(str(special.get("enter_height1", "")))
        self.enter_x1.setText(str(special.get("enter_x1", "")))
        self.enter_y1.setText(str(special.get("enter_y1", "")))
        self.enter_w2.setText(str(special.get("enter_width2", "")))
        self.enter_h2.setText(str(special.get("enter_height2", "")))
        self.enter_x2.setText(str(special.get("enter_x2", "")))
        self.enter_y2.setText(str(special.get("enter_y2", "")))
        self.camera_height.setText(str(special.get("camera_height", "")))


# 以下为模型生成函数 --------------------------------------------------------


# XML操作函数
def generate_keyboard_model(input_xml, output_xml, params, final=False):
    """主生成函数"""
    root = load_keyboard_xml(input_xml)
    coords = calculate_key_positions(params)
    save_coords(coords, output_xml)
    add_collision_elements(root, Path(input_xml).stem, coords, final=final)
    add_camera_elements(root, params["special"]["camera_height"])
    save_xml(root, output_xml)


def print_xml(root):
    """打印XML"""
    # 打印检查现有结构
    print("[XML结构预览]")
    for elem in root.iter():
        print(f"<{elem.tag} {elem.attrib}>...")


def load_keyboard_xml(xml_path):
    """加载XML文件"""
    tree = ET.parse(xml_path)
    return tree.getroot()


def save_coords(coords, output_xml):
    """将坐标信息保存为与模型文件同名的json"""
    xml_path = Path(output_xml)
    coord_path = xml_path.parent / f"keyboard_coords.json"
    
    # 转换数据结构
    coord_data = {
        key: {
            "name": key,
            "position": (float(x), float(y), float(z)),
            "size": (float(width), float(height)),
            "sensor_name": f"touch_{key}" # if key != "space" else "touch_space"
        } for key, (x, y, z, width, height) in coords.items()
    }
    
    with open(coord_path, 'w') as f:
        json.dump(coord_data, f, indent=2)


def calculate_key_positions(params):
    """计算按键坐标和尺寸，返回字典，值为 (x, y, z, width, height) 的 5 元组"""
    coords = {}
    base = params["base"]
    special = params["special"]

    # 数字行 (1-0-=)
    num_row = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "="]
    for i, key in enumerate(num_row):
        coords[key] = (
            3 * base["vertical_spacing"],  # x
            base["1_y"] - i * base["horizontal_spacing"],  # y
            0,  # z
            base["key_width"],  # width
            base["key_height"],  # height
        )

    # QWERTY行
    q_row = ["Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"]
    for i, key in enumerate(q_row):
        coords[key] = (
            2 * base["vertical_spacing"],  # x
            base["Q_y"] - i * base["horizontal_spacing"],  # y
            0,  # z
            base["key_width"],  # width
            base["key_height"],  # height
        )

    # ASDF行
    a_row = ["A", "S", "D", "F", "G", "H", "J", "K", "L", ";", "'"]
    for i, key in enumerate(a_row):
        coords[key] = (
            base["vertical_spacing"],  # x
            base["A_y"] - i * base["horizontal_spacing"],  # y
            0,  # z
            base["key_width"],  # width
            base["key_height"],  # height
        )

    # ZXCV行
    z_row = ["Z", "X", "C", "V", "B", "N", "M", ",", ".", "/"]
    for i, key in enumerate(z_row):
        coords[key] = (
            0,  # x
            -i * base["horizontal_spacing"],  # y
            0,  # z
            base["key_width"],  # width
            base["key_height"],  # height
        )

    # 空格键 (B 键下方)
    coords["space"] = (
        -base["vertical_spacing"],  # x
        -4 * base["horizontal_spacing"],  # y (与 B 键对齐)
        0,  # z
        special["space_width"],  # width
        base["key_height"],  # height
    )

    # 退格键 (= 键右侧)
    coords["backspace"] = (
        3 * base["vertical_spacing"],  # x
        special["backspace_y"],  # y
        0,  # z
        special["backspace_width"],  # width
        base["key_height"],  # height
    )

    # 回车键 (双部件)
    coords["enter1"] = (
        special["enter_x1"],  # x
        special["enter_y1"],  # y
        0,  # z
        special["enter_width1"],  # width
        special["enter_height1"],  # height
    )

    coords["enter2"] = (
        special["enter_x2"],  # x
        special["enter_y2"],  # y
        0,  # z
        special["enter_width2"],  # width
        special["enter_height2"],  # height
    )

    return coords


def add_collision_elements(root, name, coords, final=False):
    """添加碰撞体和传感器"""
    worldbody = root.find(".//worldbody")
    keyboard = worldbody.find(f"body[@name='{name}']")

    # 确保sensor节点存在
    sensor = root.find(".//sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")

    # print(worldbody)
    # print(name)
    # print(keyboard)

    # 添加新碰撞体
    for key, (x, y, z, width, height) in coords.items():
        if width == 0 or height == 0:
            continue
        if final:
            geom = ET.Element(
                "geom",
                name=f"collision_{key}",
                type="box",
                pos=f"{x:.4f} {y:.4f} {z:.4f}",
                size=f"{height:.4f} {width:.4f} 0.001",
                rgba="0.5 0.5 0.5 0.5",
                group="3",
            )
        else:
            geom = ET.Element(
                "geom",
                name=f"collision_{key}",
                type="box",
                pos=f"{x:.4f} {y:.4f} {z:.4f}",
                size=f"{height:.4f} {width:.4f} 0.001",
                rgba="0.5 0.5 0.5 0.5",
            )
        keyboard.append(geom)

        # 添加力检测传感器（需要关联site）
        site = ET.Element(
            "site",
            name=f"force_sensor_{key}",
            pos=f"{x:.4f} {y:.4f} {z:.4f}",  # 略微抬升避免接触面重合
            size="0.001 0.001 0.001",
        )
        keyboard.append(site)

        # 添加触摸传感器
        ET.SubElement(
            sensor,
            "touch",
            name=f"touch_{key}",
            site=f"force_sensor_{key}",
            cutoff="1000",  # 力检测阈值（单位：牛顿）
        )


def add_camera_elements(root, camera_height):
    """添加俯视相机"""
    worldbody = root.find(".//worldbody")

    # 创建相机配置
    camera = ET.Element(
        "camera",
        name="overhead_view",
        mode="fixed",
        pos=f"0 -0.12 {camera_height}",  # 位于键盘上方0.5米处
        quat="0.707 0 0 -0.707",  # 朝下观察（绕X轴旋转180度）
        fovy="60",  # 视野角度
    )

    # 插入到worldbody开头
    worldbody.insert(0, camera)


# def save_xml(root, output_path):
#     """保存XML文件"""
#     tree = ET.ElementTree(root)
#     tree.write(output_path, encoding='utf-8', xml_declaration=True)


def save_xml(root, output_path):
    """保存美化后的XML文件"""
    xml_str = ET.tostring(root)
    parsed = minidom.parseString(xml_str)
    with open(output_path, "w") as f:
        f.write(parsed.toprettyxml(indent="  "))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KeyboardConfigurator()
    window.show()
    sys.exit(app.exec())
