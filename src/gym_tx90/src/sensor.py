'''
Name: 
Date: 2024-04-24 22:37:46
Creator: 王飞鸿
Description: 
'''
import matplotlib.pyplot as plt
import numpy as np

# 创建空列表来存储传感器数据
sensor_data = []

# 创建图形窗口和坐标轴
fig, ax = plt.subplots()

# 创建空曲线对象
line, = ax.plot([], [])

# 设置坐标轴标签和标题
ax.set_xlabel('Time')
ax.set_ylabel('Sensor Data')
ax.set_title('Real-time Sensor Data')

# 设置曲线的初始范围
ax.set_xlim(0, 100)  # 根据需要调整范围
ax.set_ylim(0, 10)  # 根据需要调整范围

# 更新曲线数据
def update_plot(new_data):
    sensor_data.append(new_data)
    line.set_data(range(len(sensor_data)), sensor_data)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()

# 模拟实时获取传感器数据的过程
def simulate_sensor_data():
    time = 0
    while True:
        # 假设你从传感器中获取到新的数据
        new_data = np.random.uniform(0, 10)
        
        # 更新曲线图
        update_plot(new_data)
        
        # 模拟时间的推移
        time += 1
        
        # 可以根据需要进行延迟控制
        # time.sleep(delay)

        # 检查是否需要终止循环
        if time >= 1000:  # 根据需要调整循环终止条件
            break

# 启动数据可视化
plt.show(block=False)

# 启动模拟传感器数据的过程
simulate_sensor_data()