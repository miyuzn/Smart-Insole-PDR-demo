import socket
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import sensor

# 定义传感器坐标
coordinate_x_35_insole = [
    -40.6, -21.2, -6.5, 7.2, 17.3,
    -39.6, -24.3, -8.2, 4.3, 15.2,
    -35.3, -23, -8.9, 4.5, 16.2,
    -17, -8.5, 0, 8.5, 17,
    -19.5, -11, -2.5, 6, 14.5,
    -29, -19, -9, 1, 11,
    -30, -20.5, -11, -1.5, 8
]
coordinate_y_35_insole = [
    100.5, 104, 100.7, 88, 73,
    70.8, 68.9, 65, 59.8, 54,
    39, 36, 32, 28.5, 25.2,
    0, 0, 0, 0, 0,
    -40, -40, -40, -40, -40,
    -70, -70, -70, -70, -70,
    -90, -90, -90, -90, -90,
]

def update_heatmap(new_data, scatter_plot, norm):
    scatter_plot.set_offsets(np.column_stack([coordinate_x_35_insole, coordinate_y_35_insole]))
    scatter_plot.set_array(new_data)
    scatter_plot.set_norm(norm)
    scatter_plot.set_cmap('hot')
    plt.draw()
    plt.pause(0.01)

if __name__ == "__main__":
    plt.ion()
    fig, ax = plt.subplots(figsize=(4,5))
    norm = Normalize(vmin=300, vmax=1500)
    scatter_plot = ax.scatter(coordinate_x_35_insole, coordinate_y_35_insole, c=[], s=400, cmap='hot', norm=norm, marker = "s")
    plt.colorbar(scatter_plot)

    # 创建UDP socket
    local_ip = "127.0.0.1"
    local_port = 53000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))

    counter = 0

    while True:
        # 这里应该有一个函数来读取新的传感器数据
        # new_data = fetch_new_sensor_data()
        # 模拟新数据
        #new_data = np.random.randint(300, 1500, len(coordinate_x_35_insole))
        data, addr = sock.recvfrom(1024)  # 缓冲区大小为1024字节
        sensor_data = sensor.parse_sensor_data(data)
        pressure_sensors = sensor_data.pressure_sensors

        if counter % 20 == 0:
            update_heatmap(pressure_sensors, scatter_plot, norm)
    
        counter += 1
        
