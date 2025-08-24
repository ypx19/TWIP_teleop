#!/usr/bin/env python3
import json
import os
import mujoco
import numpy as np
import time
import threading
from datetime import datetime
from flask import Flask, render_template, request, jsonify, send_from_directory
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

# 路由设置
@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

# Socket.IO事件
@socketio.on('connect')
def handle_connect():
    print('客户端已连接')
    emit('sim_state', {'state': sim_data, 'image': None})

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

# 仿真线程函数
def simulation_thread():
    global sim_data, is_recording, recorded_data, sim_running, sim_slowdown, server_running
    
    # 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # 预先缓存执行器ID
    left_aid = model.actuator("left_motor").id
    right_aid = model.actuator("right_motor").id
    
    # 创建渲染器
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    # 初始化仿真数据
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
    
    # 仿真主循环
    frame_count = 0
    while server_running:
        start_time = time.time()
        
        # 计算电机控制
        forward_cmd = sim_data['left_stick_y']
        turn_cmd = sim_data['left_stick_x']
        
        # 将命令应用到左右电机
        left_cmd = forward_cmd + turn_cmd
        right_cmd = forward_cmd - turn_cmd
        
        # 直接控制轮子
        left_ctrl = left_cmd * MAX_CTRL * SPEED_SCALE
        right_ctrl = right_cmd * MAX_CTRL * SPEED_SCALE
        
        # 限幅
        left_ctrl = np.clip(left_ctrl, -MAX_CTRL, MAX_CTRL)
        right_ctrl = np.clip(right_ctrl, -MAX_CTRL, MAX_CTRL)
        
        # 写入控制
        data.ctrl[left_aid] = left_ctrl
        data.ctrl[right_aid] = right_ctrl
        
        # 更新仿真数据
        sim_data['left_motor'] = left_ctrl
        sim_data['right_motor'] = right_ctrl
        
        # 步进仿真
        mujoco.mj_step(model, data)
        
        # 获取传感器数据
        try:
            left_pos = data.sensor("left_wheel_pos").data[0]
            right_pos = data.sensor("right_wheel_pos").data[0]
            left_vel = data.sensor("left_wheel_vel").data[0]
            right_vel = data.sensor("right_wheel_vel").data[0]
        except:
            # 如果传感器不存在，尝试直接从关节获取
            try:
                left_pos = data.joint('left_wheel_joint').qpos[0]
                right_pos = data.joint('right_wheel_joint').qpos[0]
                left_vel = data.joint('left_wheel_joint').qvel[0]
                right_vel = data.joint('right_wheel_joint').qvel[0]
            except:
                left_pos = right_pos = left_vel = right_vel = 0.0
        
        # 获取车身位置和倾斜角（TWIP模型中没有这些关节，所以设置为0）
        position = 0.0
        velocity = 0.0
        tilt_angle = 0.0
        
        # 更新仿真数据
        sim_data['left_wheel_angle'] = left_pos
        sim_data['right_wheel_angle'] = right_pos
        sim_data['left_wheel_velocity'] = left_vel
        sim_data['right_wheel_velocity'] = right_vel
        sim_data['position'] = position
        sim_data['velocity'] = velocity
        sim_data['tilt_angle'] = tilt_angle
        sim_data['sim_slowdown'] = sim_slowdown
        sim_data['timestamp'] = time.time()
        
        # 渲染仿真画面
        try:
            renderer.update_scene(data, camera="main_cam")
            pixels = renderer.render()
            
            # 打印调试信息
            print(f"渲染成功，像素形状: {pixels.shape}, 数据类型: {pixels.dtype}")
            
            # 确保像素数据是正确的格式 (height, width, 3)
            if len(pixels.shape) == 3 and pixels.shape[2] == 3:
                # 将像素数据转换为base64编码的PNG图像
                img = Image.fromarray(pixels)
                buffer = BytesIO()
                img.save(buffer, format='PNG')
                img_str = base64.b64encode(buffer.getvalue()).decode()
                
                # 打印图像数据大小
                print(f"图像数据大小: {len(img_str)} 字符")
                
                # 发送仿真状态和图像数据
                socketio.emit('sim_state', {
                    'state': sim_data,
                    'image': img_str
                })
            else:
                print(f"像素数据格式不正确: {pixels.shape}")
                # 发送状态数据（不包含图像）
                socketio.emit('sim_state', {
                    'state': sim_data,
                    'image': None
                })
        except Exception as e:
            print(f"渲染失败: {e}")
            import traceback
            traceback.print_exc()
            # 发送状态数据（不包含图像）
            socketio.emit('sim_state', {
                'state': sim_data,
                'image': None
            })
        
        # 控制仿真速度
        elapsed = time.time() - start_time
        wait_time = max(0, (1.0 / 60.0) * sim_slowdown - elapsed)
        if wait_time > 0:
            time.sleep(wait_time)
        
        frame_count += 1
        if frame_count % 60 == 0:  # 每60帧打印一次
            print(f"已处理 {frame_count} 帧")

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
    # 等待仿真线程结束
    if sim_thread and sim_thread.is_alive():
        sim_thread.join(timeout=2)
    print('服务器已停止')
    sys.exit(0)

# 主函数
def main():
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 启动仿真线程
    start_simulation()
    
    # 启动Flask应用
    print("启动TWIP网页仿真器...")
    print("访问地址: http://localhost:8000")
    print("按 Ctrl+C 停止服务器")
    socketio.run(app, host='0.0.0.0', port=8000, debug=False, allow_unsafe_werkzeug=True)

if __name__ == "__main__":
    main()