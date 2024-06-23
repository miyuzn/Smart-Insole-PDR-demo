import math
import sensor_class
from sensor_class import sma_filter
import numpy as np

# 初始姿态角解算
def initial_angle(acc, mag):
    acc_x = acc[0][0]
    acc_y = acc[1][0]
    acc_z = acc[2][0]
    mx = mag[0][0]
    my = mag[1][0]
    mz = mag[2][0]
    pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2))
    roll = math.atan2(acc_y, acc_z)
    
    mx_prime = mx * math.cos(roll) + mz * math.sin(roll)
    my_prime = mx * math.sin(pitch) * math.sin(roll) + my * math.cos(pitch) - mz * math.sin(pitch) * math.cos(roll)
    
    yaw = math.atan2(-my_prime, mx_prime)
    return pitch, roll, yaw

# IMU轴映射
def adjust_imu_data(ori):
    x = 1 * ori[0]
    y = -1 * ori[2]
    z = 1 * ori[1]
    new = np.array([x,y,z])
    return new


# 加速度校正
def acc_calibration(ori_data, cali_data, z_flag = False):
    calied_data = []
    if z_flag is False:
        cali_avg = np.average(cali_data)
    else:
        cali_avg = 1 - np.average(cali_data)
    for i in range(len(ori_data)):
        calied_data.append(ori_data[i] - cali_avg)
    return calied_data

# 陀螺仪校正
def gyro_calibration(ori_data, cali_data):
    calied_data = []
    cali_avg = np.average(cali_data)
    for i in range(len(ori_data)):
        calied_data.append(ori_data[i] - cali_avg)
    return calied_data

# 磁力计校正
def mag_calibration(ori_data, cali_data):
    res = []
    # 为每个轴排序校准数据以找到最小和最大值
    m_min, m_max = sorted(cali_data), sorted(cali_data, reverse=True)

    d = 0.1  # 确定稳定最小/最大值的阈值

    # 寻找稳定最小值
    for i in range(len(m_min)-1):
        if abs(m_min[i+1] - m_min[i]) < d:
            M_MIN = m_min[i]
            break

    # 寻找每个轴的稳定最大值
    for i in range(len(m_max)-1):
        if abs(m_max[i+1] - m_max[i]) < d:
            M_MAX = m_max[i]
            break

    # 计算中心点
    center = (M_MAX + M_MIN) / 2

    # 通过计算出的中心调整磁力计数据
    for i in range(len(ori_data)):
        res.append(ori_data[i] - center)

    return res  # 返回校准后的磁力计数据

# 0速步态估计
def gait_0speed_estimate(acc, gyro, pressure=0):
    pitch_angle2 = []
    gyro2 = []
    #gyro_var = window_variance(gyro[0])
    gyro_var_np = np.diff(np.array(gyro[0]))
    gyro_var = gyro_var_np.tolist()
    gyro_var.append(0)
    gait_0speed = []
    pitch_angle_0 = math.atan(-acc[0][0] / math.sqrt(acc[1][0]**2 + acc[2][0]**2))
    for i in range(len(acc[0])):
        ax = acc[0][i]
        ay = acc[1][i]
        az = acc[2][i]
        pitch_angle2.append((math.atan(-ax / math.sqrt(ay**2 + az**2)) - pitch_angle_0) ** 2)
        gyro2.append(gyro[1][i] ** 2)

    #pitch_angle2 = sma_filter(pitch_angle2)
    gyro2 = sma_filter(gyro2, 20)

    # ?
    # for i in range(len(acc[0])):
    #     if pitch_angle2[i] >= 0.5 or gyro2[i] >= 0.3:
    #         gait_0speed.append(1)
    #     else:
    #         gait_0speed.append(0)

    for i in range(len(acc[0])):
        if gyro2[i] <= 1:
            gait_0speed.append(0)
        else:
            gait_0speed.append(1)
    gait_0speed = modify_array(gait_0speed, 20)
    return gait_0speed, pitch_angle2, gyro2

def modify_array(arr, target):
    n = len(arr)
    start = end = 0  # 初始化连续1的起始和结束位置
    while end < n:
        # 寻找连续1的序列
        if arr[end] == 1:
            start = end
            while end < n and arr[end] == 1:
                end += 1
            # 检查连续1的数量是否达到目标值
            if end - start < target:
                # 将不足目标值的1置为0
                for i in range(start, end):
                    arr[i] = 0
        else:
            end += 1
    return arr

# 求移动窗口方差
def window_variance(input, window_size=10):
    result = []

    if len(input) < window_size:
        raise ValueError("移动窗口方差错误: 数据长度小于窗口长度.")

    for i in range(len(input)):
        if i + window_size < len(input):
            tmp = input[i:i + window_size]
            result.append(np.var(tmp))
        else:
            tmp = input[i:]
            result.append(np.var(tmp))

    return result

def read_imu(file_name, max=-1):
    # 读取文件内容
    with open(file_name, 'r') as file:
        lines = file.readlines()

    # 初始化存储IMUData对象的列表
    imu_data_list = []

    # 解析IMU数据文件内容
    for line in lines:
        if max != -1 and len(imu_data_list) > max:
            return imu_data_list
        data = line.strip().split(',')
        # 解析加速度、旋转、磁力计和时间戳
        factor_a = 9.80665
        factor_g = math.pi / 180
        acceleration = [float(data[0]) * factor_a, float(data[1]) * factor_a, float(data[2]) * factor_a]
        gyro = [float(data[3]) * factor_g, float(data[4]) * factor_g, float(data[5]) * factor_g]
        magnetometer = [float(data[6]), float(data[7]), float(data[8])]
        timestamp = data[9]
        # 创建IMUData对象并添加到列表中
        imu_data = sensor_class.IMU_Data(acceleration, gyro, magnetometer, timestamp)
        imu_data_list.append(imu_data)

    return imu_data_list

def read_imu2(file_name, max=-1):
    # 读取文件内容
    with open(file_name, 'r') as file:
        lines = file.readlines()

    # 初始化存储IMUData对象的列表
    imu_data_list = []
    initial_stamp = 0
    start_flag = 0
    # 解析IMU数据文件内容
    for line in lines:
        if max != -1 and len(imu_data_list) > max:
            return imu_data_list
        if line.startswith("//") or line.startswith("PacketCounter"):
            continue
        data = line.strip().split('\t')
        # 解析加速度、旋转、磁力计和时间戳
        factor_g = 1
        acceleration = [float(data[1]), float(data[2]), float(data[3])]
        gyro = [float(data[4]) * factor_g, float(data[5]) * factor_g, float(data[6]) * factor_g]
        magnetometer = [float(data[7]), float(data[8]), float(data[9])]
        timestamp = int(data[0])
        # 更改时间戳格式
        if start_flag == 0:
            initial_stamp = timestamp
            timestamp = 0
            start_flag = 1
        else:
            timestamp -= initial_stamp
        # 创建IMUData对象并添加到列表中
        imu_data = sensor_class.IMU_Data(acceleration, gyro, magnetometer, timestamp)
        imu_data_list.append(imu_data)

    return imu_data_list



# 从IMU数据中取出某一维的数据
def get_IMU_line(imu_data_list, data_type, axis=0):
    lines = []
    index = ord(axis) - ord('x')
    if index not in [0, 1, 2]:
        raise ValueError("IMU提取函数错误：无此维度。")
    if data_type == "acc":
        for imu_data in imu_data_list:
            lines.append(imu_data.acceleration[index])
    elif data_type == "gyro":
        for imu_data in imu_data_list:
            lines.append(imu_data.gyro[index])
    elif data_type == "mag":
        for imu_data in imu_data_list:
            lines.append(imu_data.magnetometer[index])
    elif data_type == "time":
        for imu_data in imu_data_list:
            lines.append(imu_data.timestamp)
    else:
        raise TypeError("IMU提取函数错误：无此数据。")

    return lines

# IMU数据找对齐动作
def IMU_find_tiptoe_motion(acc, threshold=0.004):
    # 计算初始俯仰角
    pitch_angle_0 = math.atan(-acc[0][0] / math.sqrt(acc[1][0]**2 + acc[2][0]**2))
    start = end = 0
    for i in range(len(acc[0])):
        ax = acc[0][i]
        ay = acc[1][i]
        az = acc[2][i]
        pitch_angle2 = (math.atan(-ax / math.sqrt(ay**2 + az**2)) - pitch_angle_0) ** 2
        
        # 找开始点
        if start == 0 and pitch_angle2 >= threshold:
            start = i
        # 找结束点
        elif start > 0 and pitch_angle2 < threshold:
            end = i
            break
    print(f"IMU tiptoe start {start}; end {end}")
    return start, end

