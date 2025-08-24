import pygame
import threading
import time
from typing import Callable, Optional


class PS5Controller:
    """PS5手柄监听类，用于获取手柄输入"""
    
    def __init__(self, deadzone: float = 0.1):
        """
        初始化PS5手柄
        
        Args:
            deadzone: 摇杆死区，避免漂移
        """
        self.deadzone = deadzone
        self.joystick = None
        self.running = False
        self.thread = None
        
        # 手柄状态
        self.left_stick_x = 0.0
        self.left_stick_y = 0.0
        self.right_stick_x = 0.0
        self.right_stick_y = 0.0
        
        # 按钮状态
        self.buttons = {}
        self.button_callbacks = {}
        
        # 初始化pygame
        pygame.init()
        pygame.joystick.init()
        
        self._connect_controller()
    
    def _connect_controller(self):
        """连接PS5手柄"""
        joystick_count = pygame.joystick.get_count()
        
        if joystick_count == 0:
            print("未检测到手柄，请连接PS5手柄后重试")
            return False
        
        # 查找PS5手柄
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            
            # PS5手柄的名称通常包含"PS5"或"DualSense"
            name = joystick.get_name().lower()
            if "ps5" in name or "dualsense" in name or "wireless controller" in name:
                self.joystick = joystick
                print(f"已连接手柄: {joystick.get_name()}")
                return True
        
        # 如果没找到PS5手柄，使用第一个可用的手柄
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"已连接手柄: {self.joystick.get_name()}")
            return True
        
        return False
    
    def _apply_deadzone(self, value: float) -> float:
        """应用死区"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def update(self):
        """在主线程调用，更新手柄输入状态"""
        # 注意：在macOS上，SDL/pygame必须在主线程处理事件
        pygame.event.pump()
        
        if self.joystick:
            # 更新摇杆状态
            self.left_stick_x = self._apply_deadzone(self.joystick.get_axis(0))
            self.left_stick_y = self._apply_deadzone(-self.joystick.get_axis(1))  # 反转Y轴
            self.right_stick_x = self._apply_deadzone(self.joystick.get_axis(2))
            self.right_stick_y = self._apply_deadzone(-self.joystick.get_axis(3))  # 反转Y轴
            
            # 更新按钮状态并触发回调
            button_count = self.joystick.get_numbuttons()
            for i in range(button_count):
                button_pressed = self.joystick.get_button(i)
                button_name = f"button_{i}"
                
                if button_pressed and not self.buttons.get(button_name, False):
                    if button_name in self.button_callbacks:
                        self.button_callbacks[button_name]()
                
                self.buttons[button_name] = button_pressed
    
    def start(self):
        """兼容旧接口：不再启动后台线程，改为在主循环中调用update()"""
        print("PS5Controller: 已初始化。请在主循环中周期性调用 controller.update() 来刷新手柄状态。")
        return True
    
    def stop(self):
        """兼容旧接口：无后台线程可停止"""
        return True
    
    def register_button_callback(self, button_name: str, callback: Callable):
        """注册按钮回调函数"""
        self.button_callbacks[button_name] = callback
    
    def get_left_stick(self) -> tuple:
        """获取左摇杆状态"""
        return (self.left_stick_x, self.left_stick_y)
    
    def get_right_stick(self) -> tuple:
        """获取右摇杆状态"""
        return (self.right_stick_x, self.right_stick_y)
    
    def get_button(self, button_name: str) -> bool:
        """获取按钮状态"""
        return self.buttons.get(button_name, False)
    
    def is_connected(self) -> bool:
        """检查手柄是否已连接"""
        return self.joystick is not None
    
    def get_motor_commands(self) -> tuple:
        """
        根据手柄输入计算电机控制命令
        
        Returns:
            (left_motor, right_motor): 左右电机的控制信号 (-1.0 到 1.0)
        """
        # 使用左摇杆控制移动
        forward = self.left_stick_y  # 前进/后退
        turn = self.left_stick_x     # 左转/右转
        
        # 差分驱动控制
        left_motor = forward + turn
        right_motor = forward - turn
        
        # 限制在[-1, 1]范围内
        left_motor = max(-1.0, min(1.0, left_motor))
        right_motor = max(-1.0, min(1.0, right_motor))
        
        return (left_motor, right_motor)


if __name__ == "__main__":
    """测试PS5手柄功能（主线程循环调用update）"""
    controller = PS5Controller()
    
    if not controller.is_connected():
        print("未能连接手柄")
        exit(1)
    
    # 注册按钮回调
    def on_x_pressed():
        print("X按钮被按下")
    
    def on_square_pressed():
        print("Square按钮被按下")
    
    controller.register_button_callback("button_0", on_x_pressed)      # X按钮
    controller.register_button_callback("button_2", on_square_pressed)  # Square按钮
    
    controller.start()
    
    try:
        print("开始测试手柄，按Ctrl+C退出...")
        while True:
            controller.update()
            left_x, left_y = controller.get_left_stick()
            right_x, right_y = controller.get_right_stick()
            left_motor, right_motor = controller.get_motor_commands()
            
            print(f"\r左摇杆: ({left_x:.2f}, {left_y:.2f}) | "
                  f"右摇杆: ({right_x:.2f}, {right_y:.2f}) | "
                  f"电机: L={left_motor:.2f}, R={right_motor:.2f}", end="")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n停止测试")
    
    finally:
        controller.stop()