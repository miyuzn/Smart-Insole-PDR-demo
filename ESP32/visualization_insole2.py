import socket
import sys
import numpy as np
import pickle
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import sensor

# 定义右脚传感器坐标，起始元素对应右脚左上方的大拇指
coordinate_x_35_insole = [
    -40.6, -21.2, -6.5, 7.2, 17.3,    # 大拇指到小拇指
    -39.6, -24.3, -8.2, 4.3, 15.2,
    -35.3, -23, -8.9, 4.5, 16.2,
    -17, -8.5, 0, 8.5, 17,
    -19.5, -11, -2.5, 6, 14.5,
    -29, -19, -9, 1, 11,
    -30, -20.5, -11, -1.5, 8
]
coordinate_y_35_insole = [
    100.5, 104, 100.7, 88, 73,       # 大拇指到小拇指
    70.8, 68.9, 65, 59.8, 54,
    39, 36, 32, 28.5, 25.2,
    0, 0, 0, 0, 0,
    -40, -40, -40, -40, -40,
    -70, -70, -70, -70, -70,
    -90, -90, -90, -90, -90
]

def reorder_for_left_foot(coords_x, coords_y):
    # 将镜像后的左脚坐标从小拇指到大拇指重新排序为从大拇指到小拇指
    reordered_indices = [4, 3, 2, 1, 0,   # 小拇指到大拇指
                         9, 8, 7, 6, 5,
                         14, 13, 12, 11, 10,
                         19, 18, 17, 16, 15,
                         24, 23, 22, 21, 20,
                         29, 28, 27, 26, 25,
                         34, 33, 32, 31, 30]
    reordered_coords_x = [coords_x[i] for i in reordered_indices]
    reordered_coords_y = [coords_y[i] for i in reordered_indices]
    return reordered_coords_x, reordered_coords_y

def update_heatmap(new_data, scatter_plot, norm, coords_x, coords_y):
    scatter_plot.set_offsets(np.column_stack([coords_x, coords_y]))
    scatter_plot.set_array(new_data)
    scatter_plot.set_norm(norm)
    scatter_plot.set_cmap('hot')
    plt.draw()
    plt.pause(0.01)

if __name__ == "__main__":
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8,6))  # 创建两个子图
    norm = Normalize(vmin=300, vmax=1500)
    
    # 对左脚的x坐标进行镜像翻转并重新排序
    flipped_coordinate_x_35_insole = [-x for x in coordinate_x_35_insole]
    reordered_coords_x, reordered_coords_y = reorder_for_left_foot(flipped_coordinate_x_35_insole, coordinate_y_35_insole)
    
    scatter_plot_l = ax1.scatter(reordered_coords_x, reordered_coords_y, c=[], s=400, cmap='hot', norm=norm, marker = "s")
    scatter_plot_r = ax2.scatter(coordinate_x_35_insole, coordinate_y_35_insole, c=[], s=400, cmap='hot', norm=norm, marker = "s")
    
    # 隐藏坐标轴
    ax1.axis('off')
    ax2.axis('off')
    ax1.set_title("Left")
    ax2.set_title("Right")

    plt.colorbar(scatter_plot_l, ax=ax1)
    plt.colorbar(scatter_plot_r, ax=ax2)

    # 创建UDP socket
    local_ip = "127.0.0.1"
    local_port = 53000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))

    counter = 0


    try:
        while True:
            if plt.get_fignums() == []:  # 检查所有窗口是否都关闭
                print("窗口已关闭，程序退出中...")
                break
            
            data, addr = sock.recvfrom(1024)
            data_l, data_r = pickle.loads(data)
            sensor_data_l = sensor.parse_sensor_data(data_l)
            sensor_data_r = sensor.parse_sensor_data(data_r)
            
            pressure_sensors_l = sensor_data_l.pressure_sensors
            pressure_sensors_r = sensor_data_r.pressure_sensors

            if counter % 20 == 0:
                update_heatmap(pressure_sensors_l, scatter_plot_l, norm, reordered_coords_x, reordered_coords_y)
                update_heatmap(pressure_sensors_r, scatter_plot_r, norm, coordinate_x_35_insole, coordinate_y_35_insole)
            
            counter += 1
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        plt.close('all')  # 确保所有窗口都被关闭
        sock.close()      # 关闭套接字
        sys.exit()        # 完全退出程序