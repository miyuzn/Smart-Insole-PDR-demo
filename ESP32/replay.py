import matplotlib.pyplot as plt
import numpy as np
import csv
from time import sleep

path = "L3.csv"
row = 7
col = 5
num = row * col

# 从sensor.py导入的函数，用于读取CSV文件中的数据
def read_sensor_data_from_csv(filepath, p_num=num):
    sensor_data_list = []
    with open(filepath, 'r', newline='') as csvfile:
        # Skip the first line containing DN and SN
        next(csvfile)
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Extract timestamp and sensor values
            timestamp = float(row['Timestamp'])
            pressure_sensors = [int(row[f'P{i}']) for i in range(1, p_num + 1)]
            # Create a list of sensor data
            sensor_data_list.append(pressure_sensors)
    return sensor_data_list

# 更新热力图函数
def update_heatmap(data, heatmap, fig):
    heatmap.set_data(data)
    fig.canvas.draw()
    fig.canvas.flush_events()

# 主函数
def replay_visualization(csv_filepath):
    # 读取数据
    data_list = read_sensor_data_from_csv(csv_filepath)

    # 初始化热力图
    plt.ion()
    fig, ax = plt.subplots()
    data0 = np.zeros((row, col))
    heatmap = ax.imshow(data0, cmap='hot', interpolation='nearest')
    heatmap.set_clim(vmin=300, vmax=1500)
    plt.colorbar(heatmap)

    count = 0

    # 数据回放
    for data in data_list:
        if count % 10 == 0:
            reshaped_data = np.reshape(data, (row, col))
            update_heatmap(reshaped_data, heatmap, fig)
            sleep(0.1)  # 每帧更新间隔0.1秒
        count += 1

    plt.ioff()
    plt.show()

# 调用主函数，输入CSV文件路径
if __name__ == "__main__":
    replay_visualization(path)
