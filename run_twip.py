import time
import numpy as np
import mujoco
import pygame
from ps5_controller import PS5Controller

MODEL_PATH = 'twip_model.xml'

# 最大电机控制值（与XML中的ctrlrange对应）
MAX_CTRL = 5.0
# 直接控制参数
SPEED_SCALE = 2.0  # 速度缩放因子
# 仿真放慢倍数（值越大，仿真越慢）
SIM_SLOWDOWN = 10.0  # 默认放慢5倍


def main():
    # 声明全局变量
    global SIM_SLOWDOWN
    
    # 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # 初始化控制器
    controller = PS5Controller(deadzone=0.1)
    if not controller.is_connected():
        print("未能连接到PS5手柄，请先连接后再运行。")
        return
    controller.start()

    # 预先缓存执行器ID，避免循环中重复查询
    left_aid = model.actuator("left_motor").id
    right_aid = model.actuator("right_motor").id

    # 设置固定的渲染尺寸
    render_width, render_height = 640, 480
    # 创建一个渲染上下文，直接设置宽高
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)
    # 使用模型中的'main_cam'摄像机
    renderer.update_scene(data, camera="main_cam")
    
    # 创建一个pygame窗口，尺寸与渲染器匹配
    screen = pygame.display.set_mode((render_width, render_height))
    pygame.display.set_caption("Two Wheel Inverse Pendulum")
    font = pygame.font.Font(None, 24)
    
    # 暂停状态
    paused = False
    
    # 主循环
    running = True
    last_time = time.time()
    dt_target = model.opt.timestep  # 用MuJoCo仿真步长作为控制周期
    frame_count = 0

    try:
        while running:
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                        print("仿真" + ("暂停" if paused else "继续"))
                    elif event.key == pygame.K_UP:
                        # 加快仿真速度（减小放慢倍数）
                        SIM_SLOWDOWN = max(1.0, SIM_SLOWDOWN - 1.0)
                        print(f"仿真速度: 实时的 1/{SIM_SLOWDOWN:.1f}")
                    elif event.key == pygame.K_DOWN:
                        # 减慢仿真速度（增加放慢倍数）
                        SIM_SLOWDOWN += 1.0
                        print(f"仿真速度: 实时的 1/{SIM_SLOWDOWN:.1f}")
            
            # 在主线程刷新手柄事件
            controller.update()
            
            # 时间控制
            now = time.time()
            elapsed = now - last_time
            # 根据放慢倍数调整等待时间
            target_frame_time = dt_target * SIM_SLOWDOWN
            if elapsed < target_frame_time:
                time.sleep(target_frame_time - elapsed)
            last_time = time.time()
            
            if not paused:
                # 读取手柄左摇杆的上下移动
                left_stick_x, left_stick_y = controller.get_left_stick()
                # 只使用上下移动，忽略左右移动，确保左右电机转速一致
                forward_cmd = left_stick_y  # 上下移动控制前进/后退
                # 将同样的命令应用到左右电机
                left_cmd = right_cmd = forward_cmd
                
                # 使用手柄输入的控制信号
                    
                print(f"手柄命令: L={left_cmd:.2f}, R={right_cmd:.2f}")

                # 直接控制轮子
                left_ctrl = left_cmd * MAX_CTRL * SPEED_SCALE
                right_ctrl = right_cmd * MAX_CTRL * SPEED_SCALE
                print(f"电机控制: L={left_ctrl:.2f}, R={right_ctrl:.2f}")
                
                # 输出轮子的角度和角速度信息
                left_pos = data.sensor("left_wheel_pos").data[0]
                right_pos = data.sensor("right_wheel_pos").data[0]
                left_vel = data.sensor("left_wheel_vel").data[0]
                right_vel = data.sensor("right_wheel_vel").data[0]
                print(f"轮子角度: L={left_pos:.2f}, R={right_pos:.2f} rad")
                print(f"轮子角速度: L={left_vel:.2f}, R={right_vel:.2f} rad/s")

                # 限幅
                left_ctrl = np.clip(left_ctrl, -MAX_CTRL, MAX_CTRL)
                right_ctrl = np.clip(right_ctrl, -MAX_CTRL, MAX_CTRL)

                # 写入控制
                data.ctrl[left_aid] = left_ctrl
                data.ctrl[right_aid] = right_ctrl
                
                # 添加调试信息
                print(f"电机力矩: L={data.qfrc_actuator[model.joint('left_wheel_joint').dofadr[0]]:.2f}, R={data.qfrc_actuator[model.joint('right_wheel_joint').dofadr[0]]:.2f} N·m")
                print(f"关节约束力: L={data.qfrc_constraint[model.joint('left_wheel_joint').dofadr[0]]:.2f}, R={data.qfrc_constraint[model.joint('right_wheel_joint').dofadr[0]]:.2f} N·m")

                # 前进一步
                mujoco.mj_step(model, data)
            
            # 渲染场景并获取像素数据
            renderer.update_scene(data, camera="main_cam")
            pixels = renderer.render()
            
            # 打印像素数组的形状以进行调试（仅第一帧）
            if frame_count == 0:
                print(f"像素数组形状: {pixels.shape}, 渲染尺寸: {render_width}x{render_height}")
            
            # 确保像素数组与surface尺寸匹配
            # MuJoCo返回的像素数组形状为(height, width, 3)，而Pygame期望(width, height, 3)
            # 因此需要转置像素数组
            pixels = pixels.swapaxes(0, 1)
            
            # 创建与像素数组大小匹配的surface
            surface = pygame.Surface((pixels.shape[0], pixels.shape[1]))
            
            # 将MuJoCo渲染的像素数据显示在Pygame窗口中
            pygame.surfarray.blit_array(surface, pixels)
            screen.blit(surface, (0, 0))
            
            # 显示一些状态信息
            left_stick_x, left_stick_y = controller.get_left_stick()
            
            # 获取轮子的角度和角速度信息
            try:
                left_pos = data.sensor("left_wheel_pos").data[0]
                right_pos = data.sensor("right_wheel_pos").data[0]
                left_vel = data.sensor("left_wheel_vel").data[0]
                right_vel = data.sensor("right_wheel_vel").data[0]
            except:
                left_pos = right_pos = left_vel = right_vel = 0.0
                
            text_lines = [
                f"左摇杆: ({left_stick_x:.2f}, {left_stick_y:.2f})",
                f"电机: L={data.ctrl[left_aid]:.2f}, R={data.ctrl[right_aid]:.2f}",
                f"轮子角度: L={left_pos:.2f}, R={right_pos:.2f} rad",
                f"轮子角速度: L={left_vel:.2f}, R={right_vel:.2f} rad/s",
                f"仿真速度: 实时的 1/{SIM_SLOWDOWN:.1f}",
                "空格键: 暂停/继续  上/下箭头: 调整仿真速度  ESC: 退出"
            ]
            
            for i, line in enumerate(text_lines):
                text = font.render(line, True, (255, 255, 255))
                screen.blit(text, (10, 10 + i * 30))
            
            pygame.display.flip()
            
            # 递增帧计数
            frame_count += 1
    
    finally:
        controller.stop()
        pygame.quit()
        print("程序已退出")


if __name__ == "__main__":
    main()