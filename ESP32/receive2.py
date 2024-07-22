import socket
import signal
import sys
import sensor
import pickle
from datetime import datetime

# left:1370 right: 1371 glove:1381
PORT_L = 1370
PORT_R = 1371
data_list_l = []
data_list_r = []

# 定义一个信号处理器，用于捕获中断信号
def signal_handler(sig, frame):
    # 获取当前时间
    now = datetime.now()

    # 格式化时间
    file_name_l = str(now.strftime("%Y%m%d_%H%M%S") + "_left" + ".csv")
    file_name_r = str(now.strftime("%Y%m%d_%H%M%S") + "_right" + ".csv")

    print(f'\nExiting gracefully. Sensor data saved to {file_name_l}, {file_name_r}.')
    sensor.save_sensor_data_to_csv(data_list_l, file_name_l)
    sensor.save_sensor_data_to_csv(data_list_r, file_name_r)
    sys.exit(0)

# 配置信号处理器来监听SIGINT（通常是Ctrl+C）
signal.signal(signal.SIGINT, signal_handler)

# 接收广播数据函数
def receive_broadcast():
    udp_ip = ""  # 监听广播地址
    udp_port_l = PORT_L
    udp_port_r = PORT_R
    count = 0
    # 创建UDP socket
    # 接收数据的sock
    sock_l = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 绑定端口和IP
    sock_l.bind((udp_ip, udp_port_l))
    sock_r.bind((udp_ip, udp_port_r))

    # 用于可视化的sock
    local_ip = "127.0.0.1"
    local_port = 53000
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Connecting to sensor...")
    while True:
        data_l, addr_l = sock_l.recvfrom(1024)  # 缓冲区大小为1024字节
        data_r, addr_r = sock_r.recvfrom(1024)  # 缓冲区大小为1024字节

        data2 = pickle.dumps((data_l, data_r))
        sock2.sendto(data2, (local_ip, local_port))
        
        data_list_l.append(sensor.parse_sensor_data(data_l))
        data_list_r.append(sensor.parse_sensor_data(data_r))
        count += 1
        if count % 100 == 0:
            print(f"\rReceived {count} packets from {addr_l}, {addr_r}, Press Ctrl+C to stop receive.")

if __name__ == "__main__":
    receive_broadcast()
