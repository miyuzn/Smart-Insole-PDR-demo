import matplotlib.pyplot as plt

from pdr import *
from imu import *
from fuse import *
from pressure import *
#from main_test import align_gait
import optitrack
from shoe import *

def main():
    # 全局参数
    imu_freq = 100
    pressure_freq = 50
    read_data_length = -1
    pressure_l_offset = 0
    pressure_r_offset = 0
    raise_sensitivity = 500

    data_type = "3m-2"

    # 源数据路径
    imu_ori_path = f"data/{data_type}/imu.txt"
    gyro_cali_path = "data/imu-stable.txt"
    mag_cali_path = "data/imu-3d.txt"

    p1_path = f"data/{data_type}/p1"
    p2_path = f"data/{data_type}/p2"
    p1_cali_path = "data/insole_cali/p1"
    p2_cali_path = "data/insole_cali/p2"



    # 读入imu原始数据
    imu_ori_data = read_imu2(imu_ori_path, read_data_length)
    # 读入陀螺仪和磁场校正数据
    gyro_cali_data = read_imu2(gyro_cali_path, len(imu_ori_data))
    mag_cali_data = read_imu2(mag_cali_path, len(imu_ori_data))
    # 读入压力传感器数据
    p1 = read_pressure(p1_path)
    p2 = read_pressure(p2_path)
    # 对齐两个压力传感器
    p1, p2 = pressure_align(p1, p2)
    # 读入压力传感器修正数据
    p1_cali = read_pressure(p1_cali_path)
    p2_cali = read_pressure(p2_cali_path)

    # 压力值修正
    p1_calied = pressure_calibration(p1[pressure_l_offset:], p1_cali)
    p2_calied = pressure_calibration(p2[pressure_r_offset:], p2_cali)

    #p1_calied = p1[pressure_l_offset:]
    #p2_calied = p2[pressure_r_offset:]


    # 对齐压力传感器
    # p1_aligned, p2_aligned, pressure_tiptoe_end = shift_pressure_sensor(p1_calied, p2_calied)
    # high_peaks, low_peaks, length = find_cop_peaks(p1_aligned, p2_aligned)

    # 基于GLRT的压力传感器对齐
    #p1_aligned, p2_aligned = align_pressure_by_GLRT(p1_calied, p2_calied)
    p1_aligned, p2_aligned = p1_calied, p2_calied


    # 从cop计算步态
    #cop_gait_list = find_cop_gait(high_peaks, low_peaks, length)

    # 更新cop步态算法
    #cop_gait_list2 = find_cop_gait_2(p1_aligned, p2_aligned, raise_sensitivity)

    # 参数调整
    sma_filter_window_size = 10
    
    true_distance = 3
    imu_offset = 0
    pressure_offset_po = 0
    pressure_range = 500


    #show_pressure_cali(sum_pressure_list(p1), sum_pressure_list(p1_calied), sum_pressure_list(p2), sum_pressure_list(p2_calied))
    #show_cop(cop_list_calculate(p1_calied), cop_list_calculate(p2_calied))
    # 计算观测数据量、时间、平均频率
    #data_count = len(imu_ori_data)
    #time = imu_ori_data[-1].timestamp
    #freq_avg = data_count / float(time)
    # 提取三维加速度原始数据，并进行滤波
    acc_ori = []
    acc_fil = []
    acc_ori.append(get_IMU_line(imu_ori_data, "acc", "x"))
    acc_ori.append(get_IMU_line(imu_ori_data, "acc", "y"))
    acc_ori.append(get_IMU_line(imu_ori_data, "acc", "z"))
    for i in acc_ori:
        acc_fil.append(sma_filter(i[imu_offset:], sma_filter_window_size))
    # 提取三维陀螺仪原始数据，并进行校正和滤波
    gyro_ori = []
    gyro_cali = []
    gyro_calied = []
    gyro_fil = []
    gyro_ori.append(get_IMU_line(imu_ori_data, "gyro", "x"))
    gyro_ori.append(get_IMU_line(imu_ori_data, "gyro", "y"))
    gyro_ori.append(get_IMU_line(imu_ori_data, "gyro", "z"))
    gyro_cali.append(get_IMU_line(gyro_cali_data, "gyro", "x"))
    gyro_cali.append(get_IMU_line(gyro_cali_data, "gyro", "y"))
    gyro_cali.append(get_IMU_line(gyro_cali_data, "gyro", "z"))
    for i in range(len(gyro_ori)):
        gyro_calied.append(gyro_calibration(gyro_ori[i][imu_offset:], gyro_cali[i]))
    for i in gyro_calied:
        gyro_fil.append(sma_filter(i, sma_filter_window_size))
    # 提取三维地磁仪原始数据，并进行校正和滤波
    mag_ori = []
    mag_cali = []
    mag_calied = []
    mag_fil = []
    mag_ori.append(get_IMU_line(imu_ori_data, "mag", "x"))
    mag_ori.append(get_IMU_line(imu_ori_data, "mag", "y"))
    mag_ori.append(get_IMU_line(imu_ori_data, "mag", "z"))
    mag_cali.append(get_IMU_line(mag_cali_data, "mag", "x"))
    mag_cali.append(get_IMU_line(mag_cali_data, "mag", "y"))
    mag_cali.append(get_IMU_line(mag_cali_data, "mag", "z"))
    for i in range(len(mag_ori)):
        mag_calied.append(mag_calibration(mag_ori[i][imu_offset:], mag_cali[i]))
    for i in mag_calied:
        mag_fil.append(sma_filter(i, sma_filter_window_size))


    # 基于GLRT的imu和压力传感器对齐
    right_end, imu_first_step = align_imu_pressure_by_GLRT(acc_fil, gyro_fil, p1_calied, p2_calied, 'r', 1e6, 1e4)

    # 基于GLRT的步态算法
    fused_gait_GLRT = fused_gait_by_GLRT(acc_fil, gyro_fil, p1_aligned, p2_aligned, right_end, imu_first_step, imu_freq, pressure_freq, 'r', 2e6)

    # 基于IMU的步态算法
    gait_0speed_list, pitch_angle2, gyro2 = gait_0speed_estimate(
        acc_fil, gyro_fil)
    
    #show_gaits(gait_0speed_list, fused_gait_GLRT)

    roll = angle_update(acc_fil, gyro_fil, gait_0speed_list, imu_freq,
                        "roll")
    pitch = angle_update(acc_fil, gyro_fil, gait_0speed_list, imu_freq,
                         "pitch")
    roll2 = angle_update(acc_fil, gyro_fil, fused_gait_GLRT, imu_freq,
                        "roll")
    pitch2 = angle_update(acc_fil, gyro_fil, fused_gait_GLRT, imu_freq,
                         "pitch")
    yaw = LKF_filterpy(gyro_fil, acc_fil, mag_fil, imu_freq)

    theta_x0 = angle_update_i(acc_fil, gyro_fil[0], imu_freq)
    theta_y0 = angle_update_i(acc_fil, gyro_fil[1], imu_freq)
    #yaw = angle_update_i(acc_fil, gyro_fil[2], imu_freq)

    # 基于总压力的步态融合
    #pressure_gait_list = pressure_gait(p1_sum, p2_sum, pressure_offset_po)

    # cop步态
    # cop重采样
    #cop_gait_list_60 = re_samp(cop_gait_list, pressure_freq, imu_freq)
    #cop_gait_list_aligned = align_gait(gait_0speed_list, cop_gait_list_60)

    # cop重采样
    # 找IMU对齐动作结束点
    #imu_tiptoe_start, imu_tiptoe_end = IMU_find_tiptoe_motion(acc_fil)

    # cop重采样
    #cop_gait_list_aligned = align_gait_2(gait_0speed_list, cop_gait_list, imu_tiptoe_end, pressure_tiptoe_end, imu_freq, pressure_freq)

    #gait_fused = gait_fusion(gait_0speed_list, cop_gait_list_aligned)



    # 坐标系转换
    print("IMU:")
    acc_global = coordinate_convert(acc_fil, roll, pitch, yaw)
    velocity = get_velocity_with_0speed(acc_global, gait_0speed_list, imu_freq)
    path_list = get_path(velocity, imu_freq)
    distance = path_distance(path_list)
    error_calculate(distance, true_distance)
    path_2d = get_2d_path(path_list, yaw)
    #draw_2d_path(path_2d)
    print("GLRT:")
    acc_global2 = coordinate_convert(acc_fil, roll2, pitch2, yaw)
    velocity2 = get_velocity_with_0speed(acc_global2, fused_gait_GLRT, imu_freq)
    velocity3 = get_velocity_with_0speed(acc_global2, [1 for i in range(len(fused_gait_GLRT))], imu_freq)
    plt.plot([-i for i in velocity[1]], label="IMU velocity")
    plt.plot([-i for i in velocity2[1]], label="Fuse velocity")
    plt.plot([-i for i in velocity3[1]], label="Without ZUPT")
    plt.title("Speed (Y Axis)")
    plt.legend()
    plt.show()
    path_list2 = get_path(velocity2, imu_freq)
    distance2 = path_distance(path_list2)
    error_calculate(distance2, true_distance)
    path_2d2 = get_2d_path(path_list2, yaw)
    #draw_2d_path(path_2d2)
    path_list3 = get_path(velocity3, imu_freq)
    distance3 = path_distance(path_list3)
    #print("Only Integral:")
    #error_calculate(distance3, true_distance)
    path_2d3 = get_2d_path(path_list3, yaw)


    opti_path = optitrack.opti_test(data_type)
    optitrack.draw_2d_path2(path_2d, path_2d2, path_2d3, opti_path, "Data Path")




def test():
    #sma_test()
    #a = read_file("C:\\Users\\CNLab\\Downloads\\data\\Data_20211011\\s1250021_1m_1.log")
    #print(a)

    #read_pressure("C:\\Users\\CNLab\\OneDrive - u-aizu.ac.jp\\jlab\\data\\pressure\\p1.txt")
    imu_ori_data = read_imu("C:\\Users\\CNLab\\OneDrive - u-aizu.ac.jp\\jlab\\data\\Data_20211011\\s1250021_4m_1.log")

    a_x_ori = get_IMU_line(imu_ori_data, "acc", "x")

    #plt.plot(a_x_ori, label="ori")
    #plt.plot(a_x_sma, label="smaed")
    #plt.show()

def show_gaits(imu_gait, pressure_gait):
    plt.plot(imu_gait, label="IMU")
    plt.plot(pressure_gait, label="IMU-Pressure Fuse")
    plt.title("Zero Velocity List")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
