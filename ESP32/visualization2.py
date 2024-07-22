import socket
import matplotlib.pyplot as plt
import numpy as np
import pickle
from PIL import Image
import sensor

row = 7
col = 5


def update_heatmap(heatmap, new_data):
    heatmap.set_data(new_data)
    # 需要更新色彩条的最大最小值，如果数据变化幅度大的话
    
    fig.canvas.draw()
    fig.canvas.flush_events()
    
if __name__ == "__main__":

    # 初始化热力图
    plt.ion()  # 开启interactive mode
    fig, (ax1, ax2) = plt.subplots(1, 2)  # 创建两个子图
    data0 = np.random.randint(309, 314, (row, col))
    heatmap_l = ax1.imshow(data0, cmap='hot', interpolation='nearest')
    heatmap_l.set_clim(vmin=300, vmax=1500)
    heatmap_r = ax2.imshow(data0, cmap='hot', interpolation='nearest')
    heatmap_r.set_clim(vmin=300, vmax=1500)
    # 添加颜色条
    cbar_l = plt.colorbar(heatmap_l, ax=ax1)
    cbar_l.set_label('Value')
    cbar_r = plt.colorbar(heatmap_r, ax=ax2)
    cbar_r.set_label('Value')

    # 创建UDP socket
    local_ip = "127.0.0.1"
    local_port = 53000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))

    counter = 0

    while True:
        data, addr = sock.recvfrom(1024)  # 缓冲区大小为1024字节
        data_l, data_r = pickle.loads(data)
        sensor_data_l = sensor.parse_sensor_data(data_l)
        pressure_sensors_l = sensor_data_l.pressure_sensors
        sensor_data_r = sensor.parse_sensor_data(data_r)
        pressure_sensors_r = sensor_data_r.pressure_sensors
        #print(pressure_sensors)
        if counter % 20 == 0:
            update_heatmap(heatmap_l, np.reshape(pressure_sensors_l,(row, col)))
            update_heatmap(heatmap_r, np.reshape(pressure_sensors_r,(row, col)))
        
        counter += 1
