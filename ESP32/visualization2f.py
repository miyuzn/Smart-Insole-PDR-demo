import socket
import matplotlib.pyplot as plt
import numpy as np
import pickle
from PIL import Image
import sensor
import pandas as pd

# 读取传感器的拟合参数
def load_fitting_parameters(file_path):
    df = pd.read_csv(file_path)
    params = {row['Sensor']: (row['k'], row['alpha']) for _, row in df.iterrows()}
    return params

# 定义热图形状
row = 7
col = 5

def update_heatmap(heatmap, new_data):
    heatmap.set_data(new_data)
    # 更新颜色条的最小和最大值
    fig.canvas.draw()
    fig.canvas.flush_events()

if __name__ == "__main__":
    # 加载拟合参数（假设true_left和true_right分别为左右脚的拟合参数文件）
    size = 26
    true_left_file = f"./true/{size}/true_left.csv"  # 左侧传感器的拟合参数文件
    true_right_file = f"./true/{size}/true_right.csv"  # 右侧传感器的拟合参数文件
    params_left = load_fitting_parameters(true_left_file)
    params_right = load_fitting_parameters(true_right_file)

    # 初始化热图
    plt.ion()  # 启用交互模式
    fig, (ax1, ax2) = plt.subplots(1, 2)  # 创建两个子图
    data0 = np.random.randint(0, 1, (row, col))
    heatmap_l = ax1.imshow(data0, cmap='hot', interpolation='nearest')
    heatmap_l.set_clim(vmin=0, vmax=50)  # 假设压力值范围为0到1000牛顿
    heatmap_r = ax2.imshow(data0, cmap='hot', interpolation='nearest')
    heatmap_r.set_clim(vmin=0, vmax=50)
    # 添加颜色条
    cbar_l = plt.colorbar(heatmap_l, ax=ax1)
    cbar_l.set_label('Pressure (N)')
    cbar_r = plt.colorbar(heatmap_r, ax=ax2)
    cbar_r.set_label('Pressure (N)')

    # 创建UDP socket
    local_ip = "127.0.0.1"
    local_port = 53000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))

    counter = 0

    while True:
        data, addr = sock.recvfrom(1024)  # 缓冲区大小为1024字节
        data_l, data_r = pickle.loads(data)

        counter += 1

        # 每25个数据包才更新热图，因此仅在需要更新热图时进行转换计算
        if counter % 33 == 0:
            # 解析传感器数据
            sensor_data_l = sensor.parse_sensor_data(data_l)
            sensor_data_r = sensor.parse_sensor_data(data_r)

            # 获取左右脚的压力传感器电阻值
            pressure_sensors_l = sensor_data_l.pressure_sensors
            pressure_sensors_r = sensor_data_r.pressure_sensors

            # 将电阻值转换为压力值，并检查异常值
            for i in range(len(pressure_sensors_l)):
                sensor_id = i + 1
                if sensor_id in params_left:
                    k, alpha = params_left[sensor_id]
                    v = pressure_sensors_l[i] / 1000
                    if v <= 0.312:
                        pressure_sensors_l[i] = 0
                        continue
                    else:
                        R = 5000 * 0.312 / (v - 0.312)
                        pressure_value = (R / k) ** (1 / alpha)
                        # 检查异常值：如果压力大于100N，则设为0
                        if pressure_value > 50:
                            pressure_sensors_l[i] = 0
                        else:
                            pressure_sensors_l[i] = pressure_value

            for i in range(len(pressure_sensors_r)):
                sensor_id = i + 1
                if sensor_id in params_right:
                    k, alpha = params_right[sensor_id]
                    v = pressure_sensors_r[i] / 1000
                    if v <= 0.312:
                        pressure_sensors_r[i] = 0
                        continue
                    else:
                        R = 5000 * 0.312 / (v - 0.312)
                    if R != float('inf'):
                        pressure_value = (R / k) ** (1 / alpha)
                        # 检查异常值：如果压力大于100N，则设为0
                        if pressure_value > 50:
                            pressure_sensors_r[i] = 0
                        else:
                            pressure_sensors_r[i] = pressure_value

            # 更新热图
            update_heatmap(heatmap_l, np.reshape(pressure_sensors_l, (row, col)))
            update_heatmap(heatmap_r, np.reshape(pressure_sensors_r, (row, col)))

        
