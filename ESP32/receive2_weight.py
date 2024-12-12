import socket
import signal
import sys
import sensor
import pickle
import threading
from datetime import datetime

# 定义端口
PORT_L = 13250
PORT_R = 13251
data_list_l = []
data_list_r = []
exp_name = './exp/1115/'

# 定义用于可视化的本地端口
local_ip = "127.0.0.1"
local_port_l = 53000  # 左脚数据端口
local_port_r = 53001  # 右脚数据端口

# 定义一个信号处理器来捕获中断信号
def signal_handler(sig, frame):
    # 获取当前时间
    now = datetime.now()

    # 格式化时间
    file_name_l = exp_name + str(now.strftime("%Y%m%d_%H%M%S") + "_left" + ".csv")
    file_name_r = exp_name + str(now.strftime("%Y%m%d_%H%M%S") + "_right" + ".csv")

    print(f'\nExiting gracefully. Sensor data saved to {file_name_l}, {file_name_r}.')
    sensor.save_sensor_data_to_csv(data_list_l, file_name_l)
    sensor.save_sensor_data_to_csv(data_list_r, file_name_r)
    sys.exit(0)

# 配置信号处理器监听SIGINT（通常是Ctrl+C）
signal.signal(signal.SIGINT, signal_handler)

# 左脚数据接收和转发函数
def receive_and_forward_left():
    udp_ip = ""  # 监听广播地址
    udp_port_l = PORT_L
    count = 0

    # 创建用于接收左脚数据的UDP套接字
    sock_l = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_l.bind((udp_ip, udp_port_l))

    # 创建用于发送左脚数据到可视化程序的UDP套接字
    sock_viz_l = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Receiving left foot sensor data...")
    while True:
        data_l, addr_l = sock_l.recvfrom(1024)  # 缓冲区大小为1024字节

        # 发送左脚数据到可视化程序端口
        data_pickled = pickle.dumps(data_l)
        sock_viz_l.sendto(data_pickled, (local_ip, local_port_l))

        # 解析并存储数据
        data_list_l.append(sensor.parse_sensor_data(data_l))
        count += 1
        if count % 100 == 0:
            print(f"\rReceived {count} left foot packets from {addr_l}, Press Ctrl+C to stop receiving.", end='')

# 右脚数据接收和转发函数
def receive_and_forward_right():
    udp_ip = ""  # 监听广播地址
    udp_port_r = PORT_R
    count = 0

    # 创建用于接收右脚数据的UDP套接字
    sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_r.bind((udp_ip, udp_port_r))

    # 创建用于发送右脚数据到可视化程序的UDP套接字
    sock_viz_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Receiving right foot sensor data...")
    while True:
        data_r, addr_r = sock_r.recvfrom(1024)  # 缓冲区大小为1024字节

        # 发送右脚数据到可视化程序端口
        data_pickled = pickle.dumps(data_r)
        sock_viz_r.sendto(data_pickled, (local_ip, local_port_r))

        # 解析并存储数据
        data_list_r.append(sensor.parse_sensor_data(data_r))
        count += 1
        if count % 100 == 0:
            print(f"\rReceived {count} right foot packets from {addr_r}, Press Ctrl+C to stop receiving.", end='')

if __name__ == "__main__":
    # 启动左右脚数据接收和转发的线程
    thread_left = threading.Thread(target=receive_and_forward_left)
    thread_right = threading.Thread(target=receive_and_forward_right)

    thread_left.start()
    thread_right.start()

    # 等待线程完成（实际上它们不会，因为是无限循环，除非程序被中断）
    thread_left.join()
    thread_right.join()
