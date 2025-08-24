#!/usr/bin/env python3
import json
import os
import mujoco
import numpy as np
import time
import threading
from datetime import datetime
from flask import Flask, send_from_directory, request, jsonify
from flask_socketio import SocketIO, emit
import base64
from io import BytesIO
from PIL import Image
import signal
import sys

# 模型路径
MODEL_PATH = 'twip_model.xml'
# 最大电机控制值
MAX_CTRL = 5.0
# 速度缩放因子
SPEED_SCALE = 2.0

# 全局变量
sim_data = {}
is_recording = False
recorded_data = []
sim_running = True
sim_slowdown = 5.0
sim_thread = None
recording_start_time = 0
server_running = True

# 创建Flask应用
app = Flask(__name__, static_folder='static')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 路由：主页和数据导出
@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/api/record/data')
def get_recorded_data():
    return jsonify(recorded_data)

@app.route('/api/record/save', methods=['POST'])
def save_recording():
    global recorded_data
    data = request.json
    filename = data.get('filename', f'recording_{int(time.time())}.json')
    os.makedirs('recordings', exist_ok=True)
    filepath = os.path.join('recordings', filename)
    with open(filepath, 'w') as f:
        json.dump(recorded_data, f, indent=2)
    return jsonify({"status": "saved", "filepath": filepath})

# Socket.IO事件：实时控制与状态推送
@socketio.on('connect')
def handle_connect():
    print('客户端已连接')
    emit('sim_state', sim_data)

@socketio.on('disconnect')
def handle_disconnect():
    print('客户端已断开连接')

@socketio.on('key_press')
def handle_key_press(data):
    global sim_data
    key = data.get('key', '')
    if key == 'up':
        sim_data['left_stick_y'] = 1.0
    elif key == 'down':
        sim_data['left_stick_y'] = -1.0
    elif key == 'left':
        sim_data['left_stick_x'] = -1.0
    elif key == 'right':
        sim_data['left_stick_x'] = 1.0

@socketio.on('gamepad_control')
def handle_gamepad_control(data):
    global sim_data
    sim_data['left_stick_x'] = float(data.get('x', 0.0))
    sim_data['left_stick_y'] = float(data.get('y', 0.0))

@socketio.on('pause_sim')
def handle_pause_sim():
    global sim_running
    sim_running = False
    print('仿真已暂停')

@socketio.on('resume_sim')
def handle_resume_sim():
    global sim_running
    sim_running = True
    print('仿真已继续')

@socketio.on('set_sim_speed')
def handle_set_sim_speed(data):
    global sim_slowdown
    sim_slowdown = data.get('slowdown', 5.0)
    print(f'仿真速度设置为 1/{sim_slowdown}')

@socketio.on('start_recording')
def handle_start_recording():
    global is_recording, recorded_data, recording_start_time
    is_recording = True
    recorded_data = []
    recording_start_time = time.time()
    print("开始记录数据")

@socketio.on('stop_recording')
def handle_stop_recording():
    global is_recording
    is_recording = False
    print(f"停止记录数据，共记录 {len(recorded_data)} 个数据点")

# 仿真线程函数：持续推送仿真状态和渲染图像
def simulation_thread():
    global sim_data, is_recording, recorded_data, sim_running, sim_slowdown, server_running
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    left_aid = model.actuator("left_motor").id
    right_aid = model.actuator("right_motor").id
    renderer = mujoco.Renderer(model, height=480, width=640)
    sim_data = {
        'left_stick_y': 0.0,
        'left_stick_x': 0.0,
        'left_motor': 0.0,
        'right_motor': 0.0,
        'left_wheel_angle': 0.0,
        'right_wheel_angle': 0.0,
        'left_wheel_velocity': 0.0,
        'right_wheel_velocity': 0.0,
        'sim_slowdown': sim_slowdown,
        'position': 0.0,
        'velocity': 0.0,
        'tilt_angle': 0.0,
        'timestamp': time.time()
    }
    while server_running:
        start_time = time.time()
        if sim_running:
            # 控制逻辑
            forward_cmd = sim_data['left_stick_y']
            turn_cmd = sim_data['left_stick_x']
            left_cmd = forward_cmd + turn_cmd
            right_cmd = forward_cmd - turn_cmd
            left_ctrl = left_cmd * MAX_CTRL * SPEED_SCALE
            right_ctrl = right_cmd * MAX_CTRL * SPEED_SCALE
            left_ctrl = np.clip(left_ctrl, -MAX_CTRL, MAX_CTRL)
            right_ctrl = np.clip(right_ctrl, -MAX_CTRL, MAX_CTRL)
            data.ctrl[left_aid] = left_ctrl
            data.ctrl[right_aid] = right_ctrl
            sim_data['left_motor'] = left_ctrl
            sim_data['right_motor'] = right_ctrl
            mujoco.mj_step(model, data)
            # 传感器数据
            try:
                left_pos = data.sensor("left_wheel_pos").data[0]
                right_pos = data.sensor("right_wheel_pos").data[0]
                left_vel = data.sensor("left_wheel_vel").data[0]
                right_vel = data.sensor("right_wheel_vel").data[0]
            except:
                try:
                    left_pos = data.joint('left_wheel_joint').qpos[0]
                    right_pos = data.joint('right_wheel_joint').qpos[0]
                    left_vel = data.joint('left_wheel_joint').qvel[0]
                    right_vel = data.joint('right_wheel_joint').qvel[0]
                except:
                    left_pos = right_pos = left_vel = right_vel = 0.0
            # 车身状态（如有）
            position = 0.0
            velocity = 0.0
            tilt_angle = 0.0
            sim_data['left_wheel_angle'] = left_pos
            sim_data['right_wheel_angle'] = right_pos
            sim_data['left_wheel_velocity'] = left_vel
            sim_data['right_wheel_velocity'] = right_vel
            sim_data['position'] = position
            sim_data['velocity'] = velocity
            sim_data['tilt_angle'] = tilt_angle
            sim_data['sim_slowdown'] = sim_slowdown
            sim_data['timestamp'] = time.time()
            # 记录数据
            if is_recording:
                record_entry = {
                    'timestamp': sim_data['timestamp'],
                    'position': sim_data['position'],
                    'velocity': sim_data['velocity'],
                    'tilt_angle': sim_data['tilt_angle'],
                    'left_motor': sim_data['left_motor'],
                    'right_motor': sim_data['right_motor'],
                    'left_wheel_angle': sim_data['left_wheel_angle'],
                    'right_wheel_angle': sim_data['right_wheel_angle'],
                    'left_wheel_velocity': sim_data['left_wheel_velocity'],
                    'right_wheel_velocity': sim_data['right_wheel_velocity']
                }
                recorded_data.append(record_entry)
            # 渲染仿真画面
            try:
                renderer.update_scene(data, camera="main_cam")
                pixels = renderer.render()
                if len(pixels.shape) == 3 and pixels.shape[2] == 3:
                    img = Image.fromarray(pixels)
                    buffer = BytesIO()
                    img.save(buffer, format='PNG')
                    img_str = base64.b64encode(buffer.getvalue()).decode()
                    socketio.emit('sim_state', {
                        'state': sim_data,
                        'image': img_str
                    })
                else:
                    socketio.emit('sim_state', {
                        'state': sim_data,
                        'image': None
                    })
            except Exception as e:
                print(f"渲染失败: {e}")
                import traceback
                traceback.print_exc()
                socketio.emit('sim_state', {
                    'state': sim_data,
                    'image': None
                })
        # 控制仿真速度
        elapsed = time.time() - start_time
        wait_time = max(0, (1.0 / 60.0) * sim_slowdown - elapsed)
        if wait_time > 0:
            time.sleep(wait_time)

# 启动仿真线程
def start_simulation():
    global sim_thread
    sim_thread = threading.Thread(target=simulation_thread)
    sim_thread.daemon = True
    sim_thread.start()

# 信号处理函数
def signal_handler(sig, frame):
    global server_running
    print('收到关闭信号，正在停止服务器...')
    server_running = False
    if sim_thread and sim_thread.is_alive():
        sim_thread.join(timeout=2)
    print('服务器已停止')
    sys.exit(0)

# 主函数
def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    start_simulation()
    print("启动TWIP网页仿真器...")
    print("访问地址: http://localhost:8000")
    print("按 Ctrl+C 停止服务器")
    socketio.run(app, host='0.0.0.0', port=8000, debug=False, allow_unsafe_werkzeug=True)

if __name__ == "__main__":
    main()