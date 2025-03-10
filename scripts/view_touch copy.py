import mujoco
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import glfw
import sys

def main(model_path):
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model)

    # 初始化GLFW窗口
    if not glfw.init():
        raise Exception("GLFW初始化失败")
    window = glfw.create_window(1200, 900, "键盘力信号监测", None, None)
    glfw.make_context_current(window)

    # 获取所有touch传感器
    touch_sensors = [model.sensor(i).name for i in range(model.nsensor) if model.sensor(i).name.startswith("touch_")]
    num_sensors = len(touch_sensors)
    print(f"找到{num_sensors}个力传感器")

    # 初始化绘图
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title('实时力信号监测')
    lines = {}
    time_buffer = np.linspace(0, 5, 500)
    data_buffer = {name: np.zeros(500) for name in touch_sensors}

    # 创建绘图对象
    for i, name in enumerate(touch_sensors):
        clean_name = name.replace("touch_", "").replace("_", " ")
        lines[name], = ax.plot(time_buffer, data_buffer[name], label=clean_name)
    
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 1000)
    ax.set_xlabel('时间 (秒)')
    ax.set_ylabel('力 (N)')
    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
    plt.tight_layout()

    # 仿真循环参数
    sim_duration = 10  # 秒
    sim_steps = int(sim_duration / model.opt.timestep)
    update_interval = 10  # 每10步更新一次显示

    # 主循环
    for step in range(sim_steps):
        # 执行仿真步骤
        mujoco.mj_step(model, data)
        
        # 更新数据缓冲区
        if step % update_interval == 0:
            current_time = data.time % 5
            time_buffer = np.roll(time_buffer, -1)
            time_buffer[-1] = current_time
            
            for name in touch_sensors:
                sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
                force = data.sensordata[sensor_id]
                data_buffer[name] = np.roll(data_buffer[name], -1)
                data_buffer[name][-1] = force
                lines[name].set_ydata(data_buffer[name])
                lines[name].set_xdata(time_buffer)
            
            # 更新绘图
            ax.set_xlim(time_buffer, time_buffer[-1])
            fig.canvas.draw()
            fig.canvas.flush_events()

        # 渲染3D视图
        if glfw.window_should_close(window):
            break
        renderer.update_scene(data)
        renderer.render()
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python script.py path/to/model.xml")
        sys.exit(1)
    main(sys.argv[1])