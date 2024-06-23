import matplotlib.pyplot as plt
import math
import cmath
import numpy as np
import sensor_class
import statistics
from filterpy import kalman

# 纯积分的角更新
def angle_update_i(gyro, time_list):
    angle_list = []
    for i in range(len(gyro) - 1):
        if i == 0:
            delta_t = time_list[1] - time_list[0]
            angle_list.append((delta_t / 2) * (gyro[i] + gyro[i+1]))
            continue
        # 梯形法则，数值积分
        delta_t = time_list[i] - time_list[i-1]
        angle_list.append(angle_list[i-1] + (delta_t / 2) * (gyro[i] + gyro[i+1]))
        if i == len(gyro) - 2:
            angle_list.append(angle_list[i] + (delta_t / 2) * (gyro[i] + gyro[i]))
    return angle_list

# 基于0速的俯仰、横滚角更新
def angle_update(acc, gyro, gait_0speed_list, time_list, angle_type):
    angle_list = []
    for i in range(min(len(gyro[0]), len(gait_0speed_list)) - 1):
        ax = acc[0][i]
        ay = acc[1][i]
        az = acc[2][i]
        if i > 0 and i < min(len(gyro[0]), len(gait_0speed_list)) - 1:
            delta_t = time_list[i] - time_list[i-1]
        # 梯形法则，数值积分
        if gait_0speed_list[i] == 0:
            if angle_type == "roll":
                angle_list.append(math.atan(ay / az))
            elif angle_type == "pitch":
                angle_list.append(math.atan(-ax / math.sqrt(ay**2 + az**2)))
        elif gait_0speed_list[i] == 1:
            if i != 0:
                if angle_type == "roll":
                    angle_list.append(angle_list[i-1] + (delta_t / 2) * (gyro[0][i] + gyro[0][i+1]))
                elif angle_type == "pitch":
                    angle_list.append(angle_list[i-1] + (delta_t / 2) * (gyro[1][i] + gyro[1][i+1]))
            else:
                if angle_type == "roll":
                    angle_list.append((delta_t / 2) * (gyro[0][i] + gyro[0][i+1]))
                elif angle_type == "pitch":
                    angle_list.append((delta_t / 2) * (gyro[1][i] + gyro[1][i+1]))
    return angle_list


# 基于0速的角速度校正
def ZUPT_gyro_cali(gyro, L):
    for i in range(len(gyro)):
        if L[i] == 0:
            gyro[0][i] = 0
            gyro[1][i] = 0
            gyro[2][i] = 0
    return [gyro[0], gyro[1], gyro[2]]

def LKF_filterpy(w, a, m, fs):
    dt = 1 / fs
    N = len(m[0])

    # 使用前100个磁场强度值计算偏航角的初始估计
    mx_prime = m[0][0:400]
    my_prime = m[1][0:400]
    yaw_prime = [-math.atan2(mx_prime[i], my_prime[i]) for i in range(len(mx_prime))]
    yaw_prime_0 = statistics.mean(yaw_prime)

    yaw_0 = -math.atan2(m[0][0], m[1][0])  # 初始偏航角的估计

    # 初始化卡尔曼滤波器
    kf = kalman.KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[yaw_0 - yaw_prime_0]])  # 初始状态估计
    kf.F = np.array([[1]])  # 状态转移矩阵
    kf.H = np.array([[1]])  # 观测矩阵
    kf.P *= 1e2  # 初始状态协方差
    kf.B = np.array([dt]) # 控制输入矩阵
    kf.Q = np.array([[1e-5]])  # 过程噪声协方差
    kf.R = np.array([[1e4]])  # 观测噪声协方差
    x_res = []


    for i in range(N):
        wz = w[2][i]
        mx, my = m[0][i], m[1][i]

        # 更新控制输入：偏航速率 * 单位时间
        kf.predict(u=np.array([wz]))
        # 基于磁力传感器计算出的偏航角观测值
        kf.update(z=np.array([[-math.atan2(mx, my) - yaw_prime_0]]))

        # 存储估计值
        x_res.append(kf.x[0, 0])  # 确保访问的是数组元素而不是标量

    res = np.array(x_res)
    return x_res

def coordinate_convert(a, roll, pitch, yaw):
    a_global = []
    ax = []
    ay = []
    az = []
    for i in range(min(len(a[0]), len(roll))):
        q0 = math.cos(yaw[i]/2) * math.cos(pitch[i]/2) * math.cos(roll[i]/2) + math.sin(yaw[i]/2) * math.sin(pitch[i]/2) * math.sin(roll[i]/2)
        q1 = math.cos(yaw[i]/2) * math.cos(pitch[i]/2) * math.sin(roll[i]/2) - math.sin(yaw[i]/2) * math.sin(pitch[i]/2) * math.cos(roll[i]/2)
        q2 = math.cos(yaw[i]/2) * math.sin(pitch[i]/2) * math.cos(roll[i]/2) + math.sin(yaw[i]/2) * math.cos(pitch[i]/2) * math.sin(roll[i]/2)
        q3 = -math.cos(yaw[i]/2) * math.sin(pitch[i]/2) * math.sin(roll[i]/2) + math.sin(yaw[i]/2) * math.cos(pitch[i]/2) * math.cos(roll[i]/2)
        q = sensor_class.Quaternion(q0, q1, q2, q3)
        a_local = sensor_class.Quaternion(0, a[0][i], a[1][i], a[2][i])
        q_ = q.conjugation()
        a_global.append(q.left_multiply(a_local.left_multiply(q_)))

    for i in a_global:
        ax.append(i.q1)
        ay.append(i.q2)
        az.append(i.q3)

    return [ax, ay, az]

def coordinate_convert2(a, qList):
    a_global = []
    ax = []
    ay = []
    az = []
    for i in range(min(len(a[0]), len(qList[0]))):
        q0 = qList[0][i]
        q1 = qList[1][i]
        q2 = qList[2][i]
        q3 = qList[3][i]
        q = sensor_class.Quaternion(q0, q1, q2, q3)
        a_local = sensor_class.Quaternion(0, a[0][i], a[1][i], a[2][i])
        q_ = q.conjugation()
        a_global.append(q.left_multiply(a_local.left_multiply(q_)))

    for i in a_global:
        ax.append(i.q1)
        ay.append(i.q2)
        az.append(i.q3)

    return [ax, ay, az]

def initial_coordinate_convert(w, roll, pitch, yaw):
    pass

def get_velocity_with_0speed(a, gait_0speed_list, time_list):
    vx = [0,]
    vy = [0,]
    vz = [0,]

    for i in range(1, min(len(a[0]), len(gait_0speed_list))):
        if gait_0speed_list[i] == 0:
            vx.append(0)
            vy.append(0)
            vz.append(0)
        else:
            delta_t = time_list[i] - time_list[i-1]
            vx.append(vx[i-1] + (delta_t / 2) * (a[0][i-1] + a[0][i]))
            vy.append(vy[i-1] + (delta_t / 2) * (a[1][i-1] + a[1][i]))
            vz.append(vz[i-1] + (delta_t / 2) * (a[2][i-1] + a[2][i]))
    return [vx, vy, vz]

def get_path(v, time_list):
    sx = [0,]
    sy = [0,]
    sz = [0,]
    for i in range(1, len(v[0]) - 1):
        delta_t = time_list[i+1]-time_list[i]
        sx.append(sx[i-1] + (delta_t / 2) * (v[0][i] + v[0][i+1]))
        sy.append(sy[i-1] + (delta_t / 2) * (v[1][i] + v[1][i+1]))
        sz.append(sz[i-1] + (delta_t / 2) * (v[2][i] + v[2][i+1]))
    return [sx, sy, sz]

def error_calculate(result, true):
    print(f"估计距离:{result} m.")
    print(f"真实距离:{true} m.")
    print(f"相对误差:{abs((result - true) / true) * 100} %")

def gait_fusion(imu_gait, pressure_gait):
    pressure_o = [0] * len(imu_gait)
    for i in range(len(imu_gait)):
        if imu_gait[i] == 1 or pressure_gait[i] == 1:
            pressure_o[i] = 1
    return pressure_o

def imu_gait_count(imu_gait):
    count = 0
    for i in range(len(imu_gait)):
        if imu_gait[i] == 1 and (i == 0 or imu_gait[i-1] == 0):
            count += 1
    return count

def draw_2d_path(path_2d):
    x = []
    y = []
    for i in range(len(path_2d)):
        x.append(path_2d[i].real)
        y.append(path_2d[i].imag)

    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.plot(x, y)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid()
    plt.show()


# 计算路径距离
def path_distance(p):
    dis_list = []
    for i in range(len(p[0])-1):
        # 计算每两点之间的距离
        distance = math.sqrt((p[0][i+1] - p[0][i]) ** 2 + (p[1][i+1] - p[1][i]) ** 2)
        if i == 0:
            dis_list.append(distance)
        else:
            # 累积距离
            dis_list.append(dis_list[i-1] + distance)
    return dis_list[-1]

# 计算二维轨迹
def get_2d_path(r, yaw):
    n = len(r[0])
    p = []
    for i in range(n):
        if i == 0:
            p.append(0 + 0j)  # 初始位置设为原点
        else:
            # 计算相对位置变化
            p_prime = p[i-1] + complex(0, math.sqrt((r[0][i] - r[0][i-1]) ** 2 + (r[1][i] - r[1][i-1]) ** 2))
            # 考虑方向角变化更新位置
            p.append((p_prime - p[i-1]) * cmath.exp(complex(0, yaw[i])) + p[i-1])
    return p