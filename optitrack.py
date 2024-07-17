import pandas as pd
import matplotlib.pyplot as plt
import math

def draw_2d_path(path_2d):
    x = []
    y = []
    for i in range(len(path_2d)):
        x.append(path_2d[i].real)
        y.append(path_2d[i].imag)

    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.plot(x, y)  # 添加标记以便更清楚地看到点
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('2D Path')
    plt.grid(True)
    plt.show()

def draw_2d_path2(path1, path2, path3, path4):
    x1 = []
    x2 = []
    x3 = []
    x4 = []
    y1 = []
    y2 = []
    y3 = []
    y4 = []
    for i in range(len(path1)):
        x1.append(path1[i].real)
        y1.append(path1[i].imag)
    for i in range(len(path2)):
        x2.append(path2[i].real)
        y2.append(path2[i].imag)
    for i in range(len(path3)):
        x3.append(path3[i].real)
        y3.append(path3[i].imag)
    for i in range(len(path4)):
        x4.append(path4[i].real)
        y4.append(path4[i].imag)

    plt.figure(figsize=(4.5,4))
    plt.xlim(-2, 10)
    plt.ylim(-2, 10)
    plt.plot(x1, y1, label="IMU based ZUPT")  # 添加标记以便更清楚地看到点
    plt.plot(x2, y2, label="IMU-Pressure Fuse ZUPT")
    plt.plot(x3, y3, label="Only Integral")
    plt.plot(x4, y4, label="True Path (OptiTrack)")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('2D Path')
    plt.legend()
    plt.grid(True)
    plt.show()

def draw_2d_path3(path1, path2, path3):
    x1 = []
    x2 = []
    x3 = []
    x4 = []
    y1 = []
    y2 = []
    y3 = []
    for i in range(len(path1)):
        x1.append(path1[i].real)
        y1.append(path1[i].imag)
    for i in range(len(path2)):
        x2.append(path2[i].real)
        y2.append(path2[i].imag)
    for i in range(len(path3)):
        x3.append(path3[i].real)
        y3.append(path3[i].imag)

    plt.figure(figsize=(4.5,4))
    plt.xlim(-2, 10)
    plt.ylim(-2, 10)
    plt.plot(x1, y1, label="IMU based ZUPT")  # 添加标记以便更清楚地看到点
    plt.plot(x2, y2, label="IMU-Pressure Fuse ZUPT")
    plt.plot(x3, y3, label="Only Integral")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('2D Path')
    plt.legend()
    plt.grid(True)
    plt.show()

def rotate_points(x_list, y_list, yaw):
    # 确保x_list和y_list长度相同
    if len(x_list) != len(y_list):
        raise ValueError("x和y列表的长度必须相同")
    
    # 计算旋转矩阵的元素
    cos_theta = math.cos(yaw)
    sin_theta = math.sin(yaw)
    
    # 初始化新的坐标列表
    new_x_list = []
    new_y_list = []
    
    # 遍历每个坐标点并应用旋转矩阵
    for x, y in zip(x_list, y_list):
        x_prime = cos_theta * x - sin_theta * y
        y_prime = sin_theta * x + cos_theta * y
        new_x_list.append(x_prime)
        new_y_list.append(y_prime)
    
    return new_x_list, new_y_list

def load_and_process_data(file_path, yaw0=0):
    # 加载数据，设置正确的表头行
    data = pd.read_csv(file_path, header=6)
    # 删除不必要的列和可能包含NaN值的行
    data_clean = data[['X', 'Z']].dropna()
    # 坐标旋转
    if yaw0 != 0:
        data_rotated = data_clean
        x_rotated, z_rotated = rotate_points(data_clean['X'], data_clean['Z'], yaw0)
        data_rotated['X'] = x_rotated
        data_rotated['Z'] = z_rotated
        complex_path = data_rotated.apply(lambda row: complex(row['X'], row['Z']), axis=1).tolist()
    else:
        # 将X和Z坐标转换为复数列表
        complex_path = data_clean.apply(lambda row: complex(row['X'], row['Z']), axis=1).tolist()
    return complex_path

def calculate_total_path_length(path_2d):
    total_distance = 0
    for i in range(1, len(path_2d)):
        distance = abs(path_2d[i] - path_2d[i-1])
        total_distance += distance
    return total_distance

# 假设 path_2d 已经被定义并包含路径的复数坐标

def opti_test(file_path, yaw0=0):
    # 文件路径
    file_path = f'{file_path}'

    # 处理数据
    path_2d = load_and_process_data(file_path, yaw0)

    # 计算路径的总长度
    total_length = calculate_total_path_length(path_2d)
    print("OptiTrack长度:", total_length)

    # 绘制路径
    #draw_2d_path(path_2d)
    return path_2d

if __name__ == "__main__":
    opti_test()