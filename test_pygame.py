import pygame
import time

# 简单的测试脚本，验证pygame在macOS上的基本功能

def main():
    # 初始化pygame
    pygame.init()
    pygame.joystick.init()
    
    # 检查手柄
    joystick_count = pygame.joystick.get_count()
    print(f"检测到 {joystick_count} 个手柄")
    
    if joystick_count > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"已连接手柄: {joystick.get_name()}")
    
    # 创建一个小窗口
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("Pygame测试")
    
    running = True
    start_time = time.time()
    
    try:
        while running and time.time() - start_time < 10:  # 运行10秒
            # 在主线程处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    print(f"按键: {pygame.key.name(event.key)}")
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # 清屏
            screen.fill((0, 0, 0))
            
            # 显示一些文本
            font = pygame.font.Font(None, 36)
            text = font.render("Pygame测试", True, (255, 255, 255))
            screen.blit(text, (100, 100))
            
            # 更新显示
            pygame.display.flip()
            
            # 控制帧率
            pygame.time.delay(100)
    
    finally:
        pygame.quit()
        print("测试完成")

if __name__ == "__main__":
    main()