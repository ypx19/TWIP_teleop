#!/usr/bin/env python3
import mujoco
import numpy as np
from PIL import Image

# 加载模型
model = mujoco.MjModel.from_xml_path('twip_model.xml')
data = mujoco.MjData(model)

# 创建渲染器
renderer = mujoco.Renderer(model, height=480, width=640)

# 步进几次仿真
for i in range(10):
    mujoco.mj_step(model, data)
    
# 渲染场景
renderer.update_scene(data, camera="main_cam")
pixels = renderer.render()

print(f"像素数组形状: {pixels.shape}")
print(f"像素数据类型: {pixels.dtype}")
print(f"像素值范围: {pixels.min()} 到 {pixels.max()}")

# 保存图像到文件进行检查
img = Image.fromarray(pixels)
img.save('test_render.png')
print("渲染图像已保存为 test_render.png")