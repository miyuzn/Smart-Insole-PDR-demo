import socket
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import sensor

row = 2
col = 2

def update_bars(new_data):
    """
    更新柱状图的高度和对应的数值标签。
    """
    # 将 2D 数组拉直为 1D，方便和柱状图的 rects 一一对应
    flat_data = new_data.flatten()

    # 更新每个柱子的高度 & 对应文字标签
    for rect, label, val in zip(rects, text_labels, flat_data):
        rect.set_height(val)
        # 更新文字位置和内容
        label.set_y(val)
        label.set_text(str(val))
    
    fig.canvas.draw()
    fig.canvas.flush_events()

if __name__ == "__main__":

    # 让 matplotlib 以交互模式运行
    plt.ion()
    fig, ax = plt.subplots()

    # 初始化一些随机数据用于演示，尺寸是 row x col
    data0 = np.random.randint(309, 314, (row, col))
    flat_data0 = data0.flatten()  # 将 2D 数据拉成 1D
    
    # x 轴上有多少个柱子
    x = np.arange(len(flat_data0))  # [0,1,2,3] 对于 2x2 = 4个传感器

    # 绘制初始柱状图
    rects = ax.bar(x, flat_data0, color='skyblue')

    # 设置坐标轴标签和刻度
    ax.set_xlabel("Sensor Index")
    ax.set_ylabel("Pressure Value")
    ax.set_xticks(x)
    ax.set_xticklabels([f"S{i+1}" for i in x])  # 如 S1, S2, S3, S4
    # 如果你的压力范围比较大，可以适当调整 y 轴最大值
    ax.set_ylim(0, 1000)

    # 创建柱状图上方的文字标签
    text_labels = []
    for rect, val in zip(rects, flat_data0):
        height = rect.get_height()
        # 在柱顶部显示数值
        label = ax.text(rect.get_x() + rect.get_width()/2.0,
                        height,
                        str(val),
                        ha='center', 
                        va='bottom')
        text_labels.append(label)

    # 创建 UDP socket 并绑定
    local_ip = "127.0.0.1"
    local_port = 53000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))

    counter = 0

    while True:
        data, addr = sock.recvfrom(1024)  # 缓冲区大小为1024字节
        sensor_data = sensor.parse_sensor_data(data)
        pressure_sensors = sensor_data.pressure_sensors
        
        # 每20帧更新一次图表
        if counter % 20 == 0:
            # 假设 pressure_sensors 本身是长度4的数组，这里 reshape 回 (row, col)
            new_data = np.reshape(pressure_sensors, (row, col))
            update_bars(new_data)
        
        counter += 1
