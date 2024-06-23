import math
# 压力传感器矩阵类
class Pressure_Matrix_Data:

    def __init__(self, timestamp, pressure_matrix):
        self.timestamp = timestamp
        self.pressure_matrix = pressure_matrix

    def __repr__(self):
        return f"Pressure_Matrix_Data:(time={self.timestamp}, pressure_matrix_data={self.pressure_matrix})"

    def sum_pressure(self):
        sum = 0
        for i in self.pressure_matrix:
            for j in i:
                sum += j
        return sum
    
    def sum_pressure_line(self, start, end):
        sum = 0
        for i in range(start, end):
            for j in range(len(self.pressure_matrix[0])):
                sum += self.pressure_matrix[i][j]
        return sum

# IMU类
class IMU_Data:
    def __init__(self, acceleration, gyro, magnetometer, timestamp):
        self.acceleration = acceleration  # 加速度 (x, y, z)
        self.gyro = gyro         # 旋转 (x, y, z)
        self.magnetometer = magnetometer  # 磁力计 (x, y, z)
        self.timestamp = timestamp        # 时间戳

    def __repr__(self):
        return (f"time={self.timestamp}), "
                f"IMU_Data(acceleration={self.acceleration}, "
                f"gyro={self.gyro}, "
                f"magnetometer={self.magnetometer}")

# 四元数类
class Quaternion:
    def __init__(self, q0, q1, q2, q3):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def conjugation(self):
        return Quaternion(self.q0, -self.q1, -self.q2, -self.q3)

    def left_multiply(self, other):
        r0 = self.q0*other.q0 - self.q1*other.q1 - self.q2*other.q2 - self.q3*other.q3
        r1 = self.q0*other.q1 + self.q1*other.q0 + self.q2*other.q3 - self.q3*other.q2
        r2 = self.q0*other.q2 - self.q1*other.q3 + self.q2*other.q0 + self.q3*other.q1
        r3 = self.q0*other.q3 + self.q1*other.q2 - self.q2*other.q1 + self.q3*other.q0
        return Quaternion(r0, r1, r2, r3)

    def tolist(self):
        pass

# SMA滤波器实现
def sma_filter(input, window_size=10):
    result = []

    if len(input) < window_size:
        raise ValueError("SMA滤波器错误: 数据长度小于窗口长度.")

    # 开始部分，窗口从1增长到window_size
    for i in range(0, window_size):
        window = input[:i + 1]
        sma = sum(window) / (i + 1)
        result.append(sma)

    # 中间部分，窗口为window_size
    for i in range(window_size, len(input) - window_size):
        window = input[i:i + window_size]
        sma = sum(window) / window_size
        result.append(sma)

    # 结尾部分，窗口从window_size减小到1
    for i in range(len(input) - window_size, len(input)):
        window = input[i:]
        sma = sum(window) / (len(input) - i)
        result.append(sma)

    return result
