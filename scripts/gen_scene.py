import os
from pathlib import Path

root_dir = Path(__file__).resolve().parent.parent / "assets" / "keyboards"

print(f"📂 当前目录：{root_dir}")

# 遍历当前目录下的所有子文件夹
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)

    # 确保是目录
    if os.path.isdir(folder_path):
        mjcf_file = os.path.join(folder_path, f"{folder}_processed.xml")  # 假设 MJCF 文件名与文件夹同名

        # 检查 MJCF 文件是否存在
        if os.path.exists(mjcf_file):
            scene_content = f'''<mujoco model="scene">
    <include file="{mjcf_file}"/>
    <worldbody>
        <!-- 添加灯光 -->
        <light name="main_light" pos="0 0 5" dir="0 0 -1" diffuse="1 1 1" specular="0.3 0.3 0.3"/>
        
        <!-- 添加地面 -->
        <geom name="ground" type="plane" pos="0 0 -0.2" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    </worldbody>
</mujoco>'''

            # 生成 scene.xml，并写入文件
            scene_path = os.path.join(folder_path, "scene.xml")
            with open(scene_path, "w", encoding="utf-8") as f:
                f.write(scene_content)

            print(f"✅ 生成 {scene_path}, 引用 {mjcf_file}")
        else:
            print(f"⚠️ 跳过 {folder}，未找到 MJCF 文件 {mjcf_file}")
