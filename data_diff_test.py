import matplotlib.pyplot as plt
import numpy as np
from main_test import show_package_diff
from pressure import read_pressure

def diff_test_main():

    left_initial_offset = 0
    right_initial_offset = 0

    p1_path2 = "D:\\insole\\ESP32\\p1"
    p2_path2 = "D:\\insole\\ESP32\\p2"

    # 读入压力传感器数据
    p1 = read_pressure(p1_path2[left_initial_offset:])
    p2 = read_pressure(p2_path2[right_initial_offset:])

    # 丢包测试
    show_package_diff(p1, p2, None)

if __name__ == "__main__":
    diff_test_main()