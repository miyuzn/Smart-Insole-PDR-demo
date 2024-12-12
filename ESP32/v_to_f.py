import sensor
import os
import pandas as pd

# 读取传感器的拟合参数
def load_fitting_parameters(file_path):
    df = pd.read_csv(file_path)
    params = {row['Sensor']: (row['k'], row['alpha']) for _, row in df.iterrows()}
    return params

# 处理目录下所有csv文件，包括子目录
def v_csv_to_r(input_dir, output_dir, sensor_num, true_left_file, true_right_file):
    params_left = load_fitting_parameters(true_left_file)
    params_right = load_fitting_parameters(true_right_file)

    for dirpath, dirnames, files in os.walk(input_dir):  # 遍历输入目录及其子目录
        # 略过名为 "F" 的子目录
        if 'F' in dirnames:
            dirnames.remove('F')

        for file in files:
            if file.endswith('.csv'):
                # 生成输入文件的完整路径
                input_csv = os.path.join(dirpath, file)
                
                # 生成相应输出目录的路径
                relative_path = os.path.relpath(dirpath, input_dir)
                output_dir_with_subdir = os.path.join(output_dir, relative_path)
                os.makedirs(output_dir_with_subdir, exist_ok=True)  # 确保输出目录存在
                
                # 生成输出文件的完整路径
                output_csv = os.path.join(output_dir_with_subdir, f'{file}')
                
                # 根据文件名选择拟合参数
                if file.endswith('left.csv'):
                    params = params_left
                elif file.endswith('right.csv'):
                    params = params_right
                else:
                    continue  # 跳过不符合条件的文件
                
                # 读取并处理数据
                s = sensor.read_sensor_data_from_csv(input_csv, sensor_num)
                for i in s:
                    i.sensor_v_to_r()
                    i.sensor_r_to_f(params)  # 将电阻转换为压力
                
                # 保存处理后的数据
                sensor.save_sensor_data_to_csv(s, output_csv)

if __name__ == "__main__":
    input_dir = "./exp/1003_26"  # 输入文件目录
    output_dir = "./exp/1003_26/F"  # 输出文件目录
    size = 26
    true_left_file = f"./true/{size}/true_left.csv"  # 左侧传感器的拟合参数文件
    true_right_file = f"./true/{size}/true_right.csv"  # 右侧传感器的拟合参数文件
    v_csv_to_r(input_dir, output_dir, 35, true_left_file, true_right_file)