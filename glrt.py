import numpy as np
import matplotlib.pyplot as plt
import math

def calculate_gravity(latitude, altitude):
    # 定义常数
    g0 = 9.80665  # 标准重力加速度 (m/s^2)
    beta = 0.0053024
    gamma = 0.0000058
    R = 6371000  # 地球平均半径 (m)

    # 纬度修正
    sin_phi = math.sin(math.radians(latitude))
    g_phi = g0 * (1 + beta * sin_phi**2 - gamma * sin_phi**4)

    # 海拔修正
    g_h = g_phi * (1 - 2 * altitude / R)

    return g_h

simdata_0 = {
    'g' : calculate_gravity(37.5, 260),
    'sigma_a' : 1,
    'sigma_g' : 1 * math.pi / 180,
    'sigma_p' : 10,
    'Window_size' : 5
}

simdata_1 = {
    'g' : calculate_gravity(37.5, 260),
    'sigma_a' : 1,
    'sigma_g' : 1 * math.pi / 180,
    'sigma_p' : 10,
    'Window_size' : 1
}

# 加速度修正函数
def acc_offset(acc, window=10):
    acc_offset = np.zeros((3,len(acc[0])))
    for i in range(len(acc)):
        for j in range(len(acc[i])):
            acc_0 = np.average(acc[i][:window])
            acc_offset[i][j] = acc[i][j] - acc_0
    return acc_offset

# gyro选择器
def g_GLRT(gyro, simdata):
    # 初始重力加速度
    g = simdata['g']
    # 加速度和陀螺仪噪声方差
    sigma2_g = simdata['sigma_g'] ** 2
    W = simdata['Window_size']
    gyro_t = np.array(gyro)
    N = len(gyro[0])
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        gyro_window = gyro_t[:, k:k + W]  # Extracting a window of gyroscope data

        for l in range(W):
            gyro_point = gyro_window[:, l]
            
            T[k] += np.dot(gyro_point, gyro_point) / sigma2_g

    return T

# acc+gyro融合GLRT选择器
def ag_GLRT(acc, gyro, simdata):
    # 初始重力加速度
    g = simdata['g']
    # 加速度和陀螺仪噪声方差
    sigma2_a = simdata['sigma_a'] ** 2
    sigma2_g = simdata['sigma_g'] ** 2
    W = simdata['Window_size']
    acc_t = np.array(acc)
    gyro_t = np.array(gyro)
    N = len(acc[0])
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        acc_window = acc_t[:, k:k + W]  # Extracting a window of accelerometer data
        gyro_window = gyro_t[:, k:k + W]  # Extracting a window of gyroscope data

        ya_m = np.mean(acc_window, axis=1)
        for l in range(W):
            acc_point = acc_window[:, l]
            gyro_point = gyro_window[:, l]
            
            tmp = acc_point - g * ya_m / np.linalg.norm(ya_m)
            T[k] += np.dot(gyro_point, gyro_point) / sigma2_g + np.dot(tmp, tmp) / sigma2_a

    return T

# 压力GLRT选择器
def p_GLRT(pressure, simdata):
    # 噪声方差
    sigma2_p = simdata['sigma_p'] ** 2
    
    N = len(pressure)
    W = simdata['Window_size']
    T = np.zeros(N - W + 1)
    pressure_t = np.array(pressure)
    pressure_max = np.max(pressure)
    pressure_avg = np.average(pressure)

    for k in range(N - W + 1):
        # 从窗口中取出传感器数据

        pressure_window = pressure_t[k:k + W]

        for l in range(W):
            pressure_point = pressure_window[l]

            p = (max(0, pressure_avg - pressure_point)) ** 2 / sigma2_p
            #p = (pressure_max - pressure_point) ** 2 / sigma2_p

            T[k] += p

    return T

# acc+gyro+压力融合GLRT选择器
def agp_GLRT(acc, gyro, pressure, simdata):
    # 初始重力加速度
    g = simdata['g']
    # 加速度和陀螺仪噪声方差
    sigma2_a = simdata['sigma_a'] ** 2
    sigma2_g = simdata['sigma_g'] ** 2
    sigma2_p = simdata['sigma_p'] ** 2

    N = len(pressure)
    W = simdata['Window_size']
    T = np.zeros(N - W + 1)
    acc_t = np.array(acc)
    gyro_t = np.array(gyro)
    pressure_t = np.array(pressure)
    pressure_max = np.max(pressure)
    pressure_avg = np.average(pressure)
    
    for k in range(N - W + 1):
        # 从窗口中取出传感器数据
        acc_window = acc_t[:, k:k + W]  # Extracting a window of accelerometer data
        gyro_window = gyro_t[:, k:k + W]  # Extracting a window of gyroscope data
        pressure_window = pressure_t[k:k + W]

        ya_m = np.mean(acc_window, axis=1)

        for l in range(W):
            acc_point = acc_window[:, l]
            gyro_point = gyro_window[:, l]
            pressure_point = pressure_window[l]
            
            tmp = acc_point - g * ya_m / np.linalg.norm(ya_m)
            T[k] += np.dot(gyro_point, gyro_point) / sigma2_g + np.dot(tmp, tmp) / sigma2_a
            #p = (pressure_max - pressure_point) ** 2 / sigma2_p
            p = (max(0, pressure_avg - pressure_point)) ** 2 / sigma2_p

            T[k] += p
            
    return T


# 从选择器结果计算gait
def shoe_gait(T, threshold):
    gait = []
    for i in range(len(T)):
        if T[i] >= threshold:
            gait.append(1)
        else:
            gait.append(0)
    return gait