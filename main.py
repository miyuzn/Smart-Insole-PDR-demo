
import matplotlib.pyplot as plt
import numpy as np
from imu import *
from fuse import *
from glrt import *
from pdr import *
from shoe import *
from ESP32 import sensor
import ekf
import optitrack

def test():
    # 参数

    # 源数据路径
    path = "240710/L3.csv"
    cali_path = "240710/gyro_cali.csv"
    mag_path = "240710/mag_cali.csv"
    #opti_file_path = "240623/opt/opt1.csv"
    # 读入imu原始数据
    ori_data = sensor.read_sensor_data_from_csv(path, 35)
    gyro_cali_data = sensor.read_sensor_data_from_csv(cali_path, 35)
    mag_cali_data = sensor.read_sensor_data_from_csv(mag_path, 35)

    # 提取传感器数据列表
    acc = ori_data.get_acc()
    gyro = ori_data.get_gyro()
    mag = ori_data.get_mag()
    time = ori_data.get_timestamp()
    pressure = ori_data.get_pressure()
    pressure_sum = ori_data.get_pressure_sum()

    pressure_cop_x, pressure_cop_y = ori_data.get_pressure_cop()

    #pressure_cop_x = sma_filter(pressure_cop_x)
    #pressure_cop_y = sma_filter(pressure_cop_y)

    show_cop(pressure_cop_x, pressure_cop_y)

    acc_cali = gyro_cali_data.get_acc()
    gyro_cali = gyro_cali_data.get_gyro()
    mag_cali = mag_cali_data.get_mag()

    # 重映射imu数据
    acc_adj = adjust_imu_data(np.array(acc))
    gyro_adj = adjust_imu_data(np.array(gyro))
    mag_adj = adjust_imu_data(np.array(mag))

    # 零偏校正
    acc_calied = []
    for i in range(len(acc_adj)):
        if i < 2:
            acc_calied.append(acc_calibration(acc_adj[i], acc_cali[i]))
        else:
            acc_calied.append(acc_calibration(acc_adj[i], acc_cali[i], True))
    gyro_calied = []
    for i in range(len(gyro_adj)):
        gyro_calied.append(gyro_calibration(gyro_adj[i], gyro_cali[i]))
    mag_calied = []
    for i in range(len(mag_adj)):
        mag_calied.append(mag_calibration(mag_adj[i], mag_cali[i]))

    # 平均窗口滤波
    acc_fil = []
    for i in acc_calied:
        acc_fil.append(sma_filter(i))
    gyro_fil = []
    for i in gyro_calied:
        gyro_fil.append(sma_filter(i))
    mag_fil = []
    for i in mag_calied:
        mag_fil.append(sma_filter(i))

    # 单位转换
    gyro_final = np.array(gyro_fil) * math.pi / 180
    acc_final = np.array(acc_fil) * 9.80665

    # 初始姿态角解算
    pitch_0, roll_0, yaw_0 = initial_angle(acc_final, mag_fil)
    print(f"pitch:{pitch_0}, roll:{roll_0}, yaw:{yaw_0}")

    # GLRT方法提取0速
    Tag = ag_GLRT(acc_final, gyro_final, simdata_0)
    #Tg = g_GLRT(gyro_adj, simdata_0)
    Tp = p_GLRT(pressure_sum, simdata_0)
    Tagp = agp_GLRT(acc_final, gyro_final, pressure_sum, simdata_0)

    plt.plot(Tag, label="IMU GLRT Selector Output")
    plt.legend()
    plt.show()
    show_GLRT(Tag, Tp, Tagp)
    Lag = shoe_gait(Tag, 1e5)
    #Lg = shoe_gait(Tg, 10000)
    Lp = shoe_gait(Tp, 1e5)
    Lagp = shoe_gait(Tagp, 1e5)
    show_0speed_list(Lag, Lagp)
    #show_0speed_acc(Lp, acc_adj)

    # 基于0速的陀螺仪修正
    gyro_ZUPT_ag = ZUPT_gyro_cali(gyro_final, Lag)
    gyro_ZUPT_p = ZUPT_gyro_cali(gyro_final, Lagp)

    # 基于EKF的姿态计算
    I4 = np.eye(4)
    #I6 = np.eye(6)
    # 调整噪声参数
    Q = 1 * (10 ** (-11)) * I4
    R_a = (10 ** (-2))
    R_m = (10 ** (0))
    noise_imu = [R_a, R_a, R_a, R_m, R_m, R_m]
    #R = 1 * (10 ** (-2)) * I6
    R = np.diag(noise_imu)
    P = I4
    q_ekf_p = ekf.EKF(gyro_ZUPT_p, acc_final, mag_fil, np.array([[1], [0], [0], [0]]), time, Q, R, P, [1, 0, 0])
    q_ekf_ag = ekf.EKF(gyro_ZUPT_ag, acc_final, mag_fil, np.array([[1], [0], [0], [0]]), time, Q, R, P, [1, 0, 0])
    roll, pitch, yaw = ekf.quaternion_to_euler(q_ekf_p)
    roll_i, pitch_i, yaw_i = ekf.quaternion_to_euler(q_ekf_ag)

    # 基于积分的姿态角计算
    # pitch_i = angle_update_i(gyro_ZUPT_ag[1], time)
    # roll_i = angle_update_i(gyro_ZUPT_ag[0], time)
    # yaw_i = angle_update_i(gyro_ZUPT_ag[2], time)
    # show_imu_data(acc_adj, gyro_adj_ZUPT_p, mag_adj, time, "ZUPT gyro")
    #show_pressure(pressure, time, path)
    #show_angles(pitch_i, roll_i, yaw_i, time, "IMU")
    show_angles(pitch, roll, yaw, time, "")
    #show_acc_global(acc_fil, time, "Only Integral")

    # 坐标转换
    acc_global_p = coordinate_convert2(acc_final, q_ekf_p)
    acc_global_ag = coordinate_convert2(acc_final, q_ekf_ag)
    # acc_global = coordinate_convert(acc_adj, roll_i, pitch_i, yaw_i)
    # gyro_global = coordinate_convert(gyro_adj, roll_i, pitch_i, yaw_i)
    # mag_global = coordinate_convert(mag_adj, roll_i, pitch_i, yaw_i)
    # plt.plot(time, gyro_global[0], label="global gyro X")
    # plt.plot(time, gyro_global[1], label="global gyro Y")
    # plt.plot(time, gyro_global[2], label="global gyro Z")
    # plt.legend()
    # plt.show()

    # 映射后姿态角解算
    # 初始姿态角解算
    # pitch_1, roll_1, yaw_1 = initial_angle(acc_global, mag_global)

    # print(f"pitch:{pitch_1}, roll:{roll_1}, yaw:{yaw_1}")

    # 基于积分的姿态角计算
    # pitch_i2 = angle_update_i(gyro_global[1], time)
    # roll_i2 = angle_update_i(gyro_global[0], time)
    # yaw_i2 = angle_update_i(gyro_global[2], time)
    #show_imu_data(acc_fil, gyro_fil, mag, time, path)
    #show_pressure(pressure, time, path)
    #show_angles(pitch_i2, roll_i2, yaw_i2, time, "Only Integral")
    #show_acc_global(acc_fil, time, "Only Integral")


    # 基于0速的姿态角计算
    # pitch = angle_update(acc_adj, gyro_adj, Lp, time, "pitch")
    # roll = angle_update(acc_adj, gyro_adj, Lp, time, "roll")
    # yaw = LKF_filterpy(gyro_adj, acc_adj, mag, 100)
    # # pitch = np.array(pitch)*math.pi/180
    # # roll = np.array(roll)*math.pi/180
    # # yaw = np.array(yaw)*math.pi/180
    # show_angles(pitch, roll, yaw[:len(pitch)], time[:len(pitch)], "ZUPT")



    # plt.plot(time, gyro[0], label="ori gyro X")
    # plt.plot(time, gyro[1], label="ori gyro Y")
    # plt.plot(time, gyro[2], label="ori gyro Z")
    # plt.legend()
    # plt.show()
    # show_acc_global(acc_global, time, "ZUPT")
    # 基于0速列表计算速度
    v1 = get_velocity_with_0speed(acc_global_ag, Lag, time)
    v2 = get_velocity_with_0speed(acc_global_p, Lagp, time)
    v3 = get_velocity_with_0speed(acc_global_ag, np.full(len(Lag), 1), time)
    plt.plot(time[:len(v1[1])], v1[1], label="IMU velocity")
    plt.plot(time[:len(v1[1])], v2[1], label="Pressure velocity")
    plt.plot(time[:len(v1[1])], v3[1], label="Only Integral")
    plt.legend()
    plt.show()

    p1 = get_path(v1, time)
    p2 = get_path(v2, time)
    p3 = get_path(v3, time)

    dis1 = path_distance(p1)
    path_2d_1 = get_2d_path(p1, yaw_i)
    dis2 = path_distance(p2)
    path_2d_2 = get_2d_path(p2, yaw)
    dis3 = path_distance(p3)
    path_2d_3 = get_2d_path(p3, yaw)
    #draw_2d_path(path_2d_1)
    print(f"Len 1:{dis1}")
    #draw_2d_path(path_2d_2)
    print(f"Len 2:{dis2}")

    #opti_path = optitrack.opti_test(opti_file_path, yaw_0 + math.pi)
    #optitrack.draw_2d_path2(path_2d_1, path_2d_2, path_2d_3, opti_path)
    optitrack.draw_2d_path3(path_2d_1, path_2d_2, path_2d_3)


# 展示GLRT选择器输出
def show_GLRT(Tag, Tp, Tagp):
    plt.plot(Tag, label = "IMU")
    plt.plot(Tp, label = "Pressure")
    plt.plot(Tagp, label = "Fusion")
    plt.legend()
    plt.title("GLRT Selector Output")
    plt.show()
    
# 展示步态输出
def show_0speed_list(Lag, Lp):
    plt.plot(Lag, label = "IMU")
    plt.plot(Lp, label = "Pressure")
    plt.legend()
    plt.title("0 Speed List Output")
    plt.show()

# 展示步态输出和IMU数据
def show_0speed_acc(L, acc):
    plt.plot(L, label="0 Speed")
    plt.plot(acc[0], label="X Acc")
    plt.plot(acc[1], label="Y Acc")
    plt.plot(acc[2], label="Z Acc")
    plt.title("0 Speed List and acc")
    plt.legend()
    plt.show()

# 展示IMU数据
def show_imu_data(acc, gyro, mag, time, title):
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(time, acc[0], label="X")
    axs[0].plot(time, acc[1], label="Y")
    axs[0].plot(time, acc[2], label="Z")
    axs[0].set_title(f"{title} Acc")
    axs[0].legend()

    axs[1].plot(time, gyro[0], label="X")
    axs[1].plot(time, gyro[1], label="Y")
    axs[1].plot(time, gyro[2], label="Z")
    axs[1].set_title(f"{title} Gyro")
    axs[1].legend()

    axs[2].plot(time, mag[0], label="X")
    axs[2].plot(time, mag[1], label="Y")
    axs[2].plot(time, mag[2], label="Z")
    axs[2].set_title(f"{title} Mag")
    axs[2].legend()
    plt.tight_layout()
    plt.show()

# 展示压力
def show_pressure(pressure, time, title):
    sum_pressure = get_sum_pressure(pressure)
    plt.plot(time, sum_pressure)
    plt.title(f"{title} Pressure")
    plt.show()

# 展示姿态角
def show_angles(pitch, roll, yaw, time, title):
    plt.plot(time, pitch, label="Pitch")
    plt.plot(time, roll, label="Roll")
    plt.plot(time, yaw, label="Yaw")
    plt.title(f"{title} Angles")
    plt.ylabel("Angle(rad)")
    plt.xlabel("Timestamp")
    plt.legend()
    plt.show()

# 展示加速度
def show_acc_global(acc_global, time, title):
    align = min(len(acc_global[0]), len(time))
    time = time[:align]
    plt.plot(time, acc_global[0], label="X")
    plt.plot(time, acc_global[1], label="Y")
    plt.plot(time, acc_global[2], label="Z")
    plt.title(f"{title} Global Acc")
    plt.legend()
    plt.show()

# 展示重心
def show_cop(x_cop, y_cop):
    plt.plot(x_cop, label="X COP")
    plt.plot(y_cop, label="Y COP")
    plt.title("COP")
    plt.legend()
    plt.show()

# 求总压力
def get_sum_pressure(pressure_sensors):
    sum_pressure_list = []
    for i in pressure_sensors:
        sum_pressure_list.append(np.sum(i))
    return sum_pressure_list


if __name__ =="__main__":
    test()
