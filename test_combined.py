import mujoco
import pygame
import time
import numpy as np

# 简化版的MuJoCo和pygame结合测试

def main():
    # 初始化pygame
    pygame.init()
    pygame.joystick.init()
    
    # 检查手柄
    joystick_count = pygame.joystick.get_count()
    print(f"检测到 {joystick_count} 个手柄")
    
    joystick = None
    if joystick_count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"已连接手柄: {joystick.get_name()}")
    
    # 加载MuJoCo模型
    model = mujoco.MjModel.from_xml_path('twip_model.xml')
    data = mujoco.MjData(model)
    
    # 预先缓存关节与执行器ID
    tilt_jid = model.joint("pendulum_tilt").id
    left_aid = model.actuator("left_motor").id
    right_aid = model.actuator("right_motor").id
    
    # 创建一个渲染上下文
    renderer = mujoco.Renderer(model)
    
    # 创建一个pygame窗口用于显示状态
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("MuJoCo + Pygame测试")
    font = pygame.font.Font(None, 24)
    
    # 最大电机控制值
    MAX_CTRL = 10.0
    
    # 简单的平衡控制器参数
    Kp_balance = 30.0
    Kd_balance = 2.0
    
    # 主循环
    running = True
    last_time = time.time()
    dt_target = model.opt.timestep
    
    try:
        while running:
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # 时间控制
            now = time.time()
            elapsed = now - last_time
            if elapsed < dt_target:
                time.sleep(dt_target - elapsed)
            last_time = time.time()
            
            # 读取手柄输入
            left_stick_x, left_stick_y = 0.0, 0.0
            if joystick:
                left_stick_x = joystick.get_axis(0)
                left_stick_y = -joystick.get_axis(1)  # 反转Y轴
                
                # 应用死区
                deadzone = 0.1
                if abs(left_stick_x) < deadzone:
                    left_stick_x = 0.0
                if abs(left_stick_y) < deadzone:
                    left_stick_y = 0.0
            
            # 差分驱动控制
            forward = left_stick_y
            turn = left_stick_x
            left_cmd = forward + turn
            right_cmd = forward - turn
            
            # 限制在[-1, 1]范围内
            left_cmd = max(-1.0, min(1.0, left_cmd))
            right_cmd = max(-1.0, min(1.0, right_cmd))
            
            # 简单的平衡控制
            theta = data.qpos[tilt_jid]
            dtheta = data.qvel[tilt_jid]
            balance = -Kp_balance * theta - Kd_balance * dtheta
            
            # 将平衡控制叠加到左右电机命令中
            left_ctrl = left_cmd * MAX_CTRL + balance
            right_ctrl = right_cmd * MAX_CTRL + balance
            
            # 限幅
            left_ctrl = np.clip(left_ctrl, -MAX_CTRL, MAX_CTRL)
            right_ctrl = np.clip(right_ctrl, -MAX_CTRL, MAX_CTRL)
            
            # 写入控制
            data.ctrl[left_aid] = left_ctrl
            data.ctrl[right_aid] = right_ctrl
            
            # 前进一步
            mujoco.mj_step(model, data)
            
            # 渲染MuJoCo场景（仅用于计算，不显示）
            renderer.update_scene(data)
            
            # 更新pygame窗口
            screen.fill((0, 0, 0))
            
            # 显示一些状态信息
            text_lines = [
                f"左摇杆: ({left_stick_x:.2f}, {left_stick_y:.2f})",
                f"电机: L={left_ctrl:.2f}, R={right_ctrl:.2f}",
                f"角度: {theta:.2f}, 角速度: {dtheta:.2f}",
                "按ESC退出"
            ]
            
            for i, line in enumerate(text_lines):
                text = font.render(line, True, (255, 255, 255))
                screen.blit(text, (10, 10 + i * 30))
            
            pygame.display.flip()
    
    finally:
        pygame.quit()
        print("测试完成")

if __name__ == "__main__":
    main()