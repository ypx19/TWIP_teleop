# Two Wheel Inverse Pendulum (TWIP) 平衡车控制

一个使用MuJoCo和PS5手柄的可交互两轮倒立摆（平衡车）仿真环境。

## 功能特性

- 🎮 **PS5手柄控制**: 使用PS5手柄的左摇杆控制平衡车的移动和转向
- 🤖 **Two Wheel Inverse Pendulum模型**: 完整的物理仿真模型，包含车体、双轮和传感器
- 🎯 **MuJoCo可视化**: 实时3D仿真显示，支持交互式观察
- ⚖️ **平衡控制**: 内置PD控制器自动维持平衡车的稳定

## 安装依赖

```bash
pip install -r requirements.txt
```

## 模块说明

### 1. `twip_model.xml` - Two Wheel Inverse Pendulum模型
- 完整的MuJoCo XML模型定义
- 包含主体、左右轮、关节、传感器和执行器
- 合理的物理参数和材质设置

### 2. `ps5_controller.py` - PS5手柄监听类
- 使用pygame库监听PS5手柄输入
- 支持摇杆死区设置，避免漂移
- 提供按钮回调功能
- 输出差分驱动控制命令

### 3. `run_twip.py` - 主控制程序
- 加载MuJoCo模型和启动可视化
- 整合手柄输入和平衡控制
- 实时控制仿真运行

## 使用方法

1. **连接PS5手柄**: 确保PS5手柄已通过蓝牙或USB连接到电脑

2. **运行仿真**:
   ```bash
   # 在macOS上需要使用mjpython而不是python
   mjpython run_twip.py
   
   # 如果不是macOS，可以直接使用python
   python run_twip.py
   ```

3. **控制说明**:
   - **左摇杆上下**: 前进/后退
   - **左摇杆左右**: 左转/右转
   - **空格键**: 暂停/恢复仿真
   - **ESC键**: 退出程序

## 控制原理

### 差分驱动
手柄输入通过差分驱动算法转换为左右轮的控制信号：
```python
forward = 左摇杆Y轴
turn = 左摇杆X轴
left_motor = forward + turn
right_motor = forward - turn
```

### 平衡控制
使用简单的PD控制器维持倒立摆的平衡：
```python
balance = -Kp * 摆角 - Kd * 摆角速度
```

## 参数调节

### 手柄参数
- `deadzone`: 摇杆死区，默认0.1
- 在`ps5_controller.py`中修改

### 控制器参数
- `Kp_balance`: 比例控制增益，默认30.0
- `Kd_balance`: 微分控制增益，默认2.0
- `MAX_CTRL`: 最大控制输出，默认10.0
- 在`run_twip.py`中修改

### 物理参数
- 车体质量、尺寸
- 轮子半径、质量
- 摩擦系数
- 在`twip_model.xml`中修改

## 系统要求

- Python 3.7+
- MuJoCo 2.3.3+
- pygame 2.1.0+
- numpy 1.21.0+
- PS5手柄（或其他兼容的游戏手柄）

## 注意事项

1. **macOS用户**: 必须使用`mjpython`命令而不是`python`来运行程序
2. **手柄连接**: 确保手柄在程序启动前已正确连接
3. **控制调节**: 可根据实际需要调整平衡控制器参数
4. **性能**: 仿真运行在实时模式下，确保系统性能足够

## 故障排除

### 手柄无法识别
- 检查手柄是否正确连接
- 尝试重新配对蓝牙连接
- 查看系统的游戏控制器设置

### 仿真卡顿
- 降低仿真频率
- 关闭其他占用GPU的程序
- 检查MuJoCo版本兼容性

### 平衡效果不佳
- 调整PD控制器参数（Kp_balance, Kd_balance）
- 检查模型的物理参数设置
- 确认初始状态是否合理

## 扩展功能

可以进一步添加的功能：
- 更复杂的控制算法（LQR、模型预测控制等）
- 多种传感器仿真（IMU、编码器等）
- 路径跟踪功能
- 障碍物环境
- 数据记录和分析