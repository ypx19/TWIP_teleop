import mujoco
from mujoco import viewer
import time

# 简单的测试脚本，验证MuJoCo基本功能

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('twip_model.xml')
    data = mujoco.MjData(model)
    
    # 简单的键盘回调
    def key_callback(keycode):
        try:
            ch = chr(keycode)
            print(f"按键: {ch}")
        except ValueError:
            return
    
    print("启动MuJoCo查看器...")
    # 使用标准方式启动查看器
    viewer.launch(model, data, key_callback=key_callback)
    print("查看器已关闭")

if __name__ == "__main__":
    main()