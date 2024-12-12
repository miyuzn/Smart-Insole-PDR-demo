import socket
import signal
import sys
import sensor
import pickle
from datetime import datetime
import pandas as pd

# 左右侧端口号
PORT_L = 13250
PORT_R = 13251

# 初始化数据存储
data_list_l = []
data_list_r = []
exp_name = './exp/test/'

# 定义信号处理函数，用于保存数据
def signal_handler(sig, frame):
    now = datetime.now()
    file_name = exp_name + str(now.strftime("%Y%m%d_%H%M%S") + "_merged.csv")
    print(f'\nExiting gracefully. Merged sensor data saved to {file_name}.')

    # 将数据转换为DataFrame
    df_l = pd.DataFrame([{
        'Timestamp': d.timestamp,
        **{f'L_P{i + 1}': p for i, p in enumerate(d.pressure_sensors)},
        'L_Mag_x': d.magnetometer[0], 'L_Mag_y': d.magnetometer[1], 'L_Mag_z': d.magnetometer[2],
        'L_Gyro_x': d.gyroscope[0], 'L_Gyro_y': d.gyroscope[1], 'L_Gyro_z': d.gyroscope[2],
        'L_Acc_x': d.accelerometer[0], 'L_Acc_y': d.accelerometer[1], 'L_Acc_z': d.accelerometer[2]
    } for d in data_list_l])

    df_r = pd.DataFrame([{
        'Timestamp': d.timestamp,
        **{f'R_P{i + 1}': p for i, p in enumerate(d.pressure_sensors)},
        'R_Mag_x': d.magnetometer[0], 'R_Mag_y': d.magnetometer[1], 'R_Mag_z': d.magnetometer[2],
        'R_Gyro_x': d.gyroscope[0], 'R_Gyro_y': d.gyroscope[1], 'R_Gyro_z': d.gyroscope[2],
        'R_Acc_x': d.accelerometer[0], 'R_Acc_y': d.accelerometer[1], 'R_Acc_z': d.accelerometer[2]
    } for d in data_list_r])

    # 对齐左右数据，按Timestamp列进行merge_asof
    merged_df = pd.merge_asof(
        df_l.sort_values("Timestamp"),
        df_r.sort_values("Timestamp"),
        on="Timestamp",
        tolerance=0.02,
        direction="nearest"
    )

    # 按提供格式排序列
    cols = (
        ['Timestamp'] +
        [f'L_P{i}' for i in range(1, 36)] +
        ['L_Mag_x', 'L_Mag_y', 'L_Mag_z', 'L_Gyro_x', 'L_Gyro_y', 'L_Gyro_z', 'L_Acc_x', 'L_Acc_y', 'L_Acc_z'] +
        [f'R_P{i}' for i in range(1, 36)] +
        ['R_Mag_x', 'R_Mag_y', 'R_Mag_z', 'R_Gyro_x', 'R_Gyro_y', 'R_Gyro_z', 'R_Acc_x', 'R_Acc_y', 'R_Acc_z']
    )
    merged_df = merged_df[cols]

    # 保存合并后的数据到CSV
    merged_df.to_csv(file_name, index=False)
    sys.exit(0)

# 捕获SIGINT信号
signal.signal(signal.SIGINT, signal_handler)

# 数据接收函数
def receive_broadcast():
    udp_ip = ""  # 监听广播地址
    udp_port_l = PORT_L
    udp_port_r = PORT_R

    # 创建UDP套接字
    sock_l = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_l.bind((udp_ip, udp_port_l))
    sock_r.bind((udp_ip, udp_port_r))

    local_ip = "127.0.0.1"
    local_port = 53000
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Connecting to sensor...")
    count = 0
    while True:
        # 接收左右侧数据
        data_l, addr_l = sock_l.recvfrom(1024)
        data_r, addr_r = sock_r.recvfrom(1024)

        # 转发数据用于可视化
        data2 = pickle.dumps((data_l, data_r))
        sock2.sendto(data2, (local_ip, local_port))
        
        # 解析并存储数据
        parsed_l = sensor.parse_sensor_data(data_l)
        parsed_r = sensor.parse_sensor_data(data_r)

        if parsed_l:
            data_list_l.append(parsed_l)
        if parsed_r:
            data_list_r.append(parsed_r)

        count += 1
        if count % 100 == 0:
            print(f"\rReceived {count} packets from {addr_l}, {addr_r}. Press Ctrl+C to stop receiving.", end='')

if __name__ == "__main__":
    receive_broadcast()
