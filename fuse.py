import numpy as np
from scipy.interpolate import interp1d
from imu import *
from glrt import *

pressure_threshold_default=10000

# 求差分
def diff_list(input_list):
    data = np.array(input_list)
    diff = np.diff(data)
    diff = np.append(diff, 0)
    return diff.tolist()

# 重采样
def re_samp(input_list, input_freq, output_freq):
    original_data = input_list
    # 定义时间轴
    duration = len(original_data) / input_freq  # 原始数据的持续时间

    # 修正的时间轴
    original_time = np.linspace(0, duration, len(original_data))
    new_time = np.linspace(0, duration, int(duration * output_freq))  # 基于时间和输出频率
    # 使用线性插值
    interpolator = interp1d(original_time, original_data, kind='linear')
    re_samp_data = interpolator(new_time)
    return re_samp_data.tolist()

# 把COP gait对齐到imu的时间轴，以第一步为对齐点
# 因为运动基于imu计算
def align_gait(imu_gait_list, cop_gait_list):
    imu_first_step = 0
    cop_first_step = 0
    for i in range(len(imu_gait_list)):
        if imu_gait_list[i] == 1:
            imu_first_step = i
            break

    for i in range(len(cop_gait_list)):
        if cop_gait_list[i] == 1:
            cop_first_step = i
            break
    
    if cop_first_step >= imu_first_step:
        cop_gait_list_aligned = cop_gait_list[cop_first_step-imu_first_step:]
    else:
        cop_gait_list_aligned = [0] * (imu_first_step-cop_first_step) + cop_gait_list

    return cop_gait_list_aligned[:len(imu_gait_list)]

# 对齐算法2
# 把COP gait对齐到imu的时间轴，以对齐动作为对齐点
def align_gait_2(imu_gait_list, cop_gait_list, imu_end, pressure_end, imu_freq, pressure_freq):
    # 重新定位压力终点
    imu_pressure_ratio = imu_freq / pressure_freq
    pressure_end_resamped = int(pressure_end * imu_pressure_ratio)
    # 重采样
    cop_gait_list_resamped = re_samp(cop_gait_list, pressure_freq, imu_freq)
    if pressure_end_resamped >= imu_end:
        cop_gait_list_aligned = cop_gait_list_resamped[pressure_end_resamped-imu_end:]
    else:
        cop_gait_list_aligned = [0] * (imu_end-pressure_end_resamped) + cop_gait_list_resamped

    return cop_gait_list_aligned[:len(imu_gait_list)]

