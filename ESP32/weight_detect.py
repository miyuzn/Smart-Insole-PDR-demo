import socket
import numpy as np
import pickle
import sensor
import time
import threading
import tkinter as tk
from tkinter import font

# 定义形状
row = 7
col = 5

class MaxDetectionApp:
    def __init__(self, master):
        self.master = master
        master.title("Max Detection")

        # 字体设置，调大字体
        self.large_font = font.Font(size=32, weight='bold')
        self.normal_font = font.Font(size=18)
        self.button_font = font.Font(size=14)

        # 主框架
        main_frame = tk.Frame(master)
        main_frame.pack()

        # 状态标签
        self.label_status = tk.Label(main_frame, text="Waiting for Recording", font=self.normal_font)
        self.label_status.pack(pady=10)

        # 按钮框架
        buttons_frame = tk.Frame(main_frame)
        buttons_frame.pack(pady=10)

        # 左右脚阈值录入按钮
        # 左脚阈值按钮
        left_buttons_frame = tk.Frame(buttons_frame)
        left_buttons_frame.pack(side='left', padx=10)
        tk.Label(left_buttons_frame, text="Left Foot", font=self.normal_font).pack()
        for i in range(1, 4):
            Wn = f'W{i}'
            btn = tk.Button(
                left_buttons_frame,
                text=f"Set Left {Wn}",
                command=lambda Wn=Wn: self.start_threshold_recording('left', Wn),
                font=self.button_font
            )
            btn.pack(pady=2)

        # 右脚阈值按钮
        right_buttons_frame = tk.Frame(buttons_frame)
        right_buttons_frame.pack(side='left', padx=10)
        tk.Label(right_buttons_frame, text="Right Foot", font=self.normal_font).pack()
        for i in range(1, 4):
            Wn = f'W{i}'
            btn = tk.Button(
                right_buttons_frame,
                text=f"Set Right {Wn}",
                command=lambda Wn=Wn: self.start_threshold_recording('right', Wn),
                font=self.button_font
            )
            btn.pack(pady=2)

        # 添加“Reset”按钮
        self.reset_button = tk.Button(buttons_frame, text="Reset", command=self.reset_thresholds, font=self.button_font)
        self.reset_button.pack(side='left', padx=10)

        # 画布框架
        canvases_frame = tk.Frame(main_frame)
        canvases_frame.pack(pady=20)

        # 左脚画布
        self.left_canvas = tk.Canvas(canvases_frame, width=220, height=400)
        self.left_canvas.pack(side='left', padx=20)
        self.left_canvas.create_text(110, 20, text="Left", font=self.normal_font)

        # 右脚画布
        self.right_canvas = tk.Canvas(canvases_frame, width=220, height=400)
        self.right_canvas.pack(side='left', padx=20)
        self.right_canvas.create_text(110, 20, text="Right", font=self.normal_font)

        # “Over Weight!”标签
        self.overweight_label = tk.Label(main_frame, text="", font=self.large_font)
        self.overweight_label.pack(pady=20)

        # 状态变量
        self.recording_left = False
        self.recording_right = False
        self.current_left_threshold = None
        self.current_right_threshold = None
        self.thresholds_left = {}   # 存储左脚的阈值，格式 {'W1': value, ...}
        self.thresholds_right = {}  # 存储右脚的阈值，格式 {'W1': value, ...}
        self.recording_start_time = None
        self.left_pressure_values = []
        self.right_pressure_values = []

        # 定义压力的最小值和最大值，用于缩放条形图
        self.pressure_min = 300
        self.pressure_max = 1000

        # 用于限制绘图速率
        self.last_update_time = time.time()  # 上一次更新绘图的时间戳
        self.update_interval = 1 / 10  # 目标帧率，10 FPS

        # 启动套接字线程
        self.sock_thread = threading.Thread(target=self.socket_thread)
        self.sock_thread.daemon = True
        self.sock_thread.start()

    def start_threshold_recording(self, foot, Wn):
        if foot == 'left' and not self.recording_left:
            self.recording_left = True
            self.current_left_threshold = Wn
            self.left_pressure_values = []
            self.recording_start_time = time.time()
            self.update_status_label(f'Recording Left {Wn}')
        elif foot == 'right' and not self.recording_right:
            self.recording_right = True
            self.current_right_threshold = Wn
            self.right_pressure_values = []
            self.recording_start_time = time.time()
            self.update_status_label(f'Recording Right {Wn}')

    # 重置阈值的方法
    def reset_thresholds(self):
        self.thresholds_left = {}
        self.thresholds_right = {}
        self.recording_left = False
        self.recording_right = False
        self.current_left_threshold = None
        self.current_right_threshold = None
        self.left_pressure_values = []
        self.right_pressure_values = []
        self.update_status_label('Thresholds Reset')
        self.update_overweight_label(False)
        # 清除画布上的阈值线
        self.clear_canvases()

    def clear_canvases(self):
        def update():
            # 清除左脚画布
            self.left_canvas.delete('all')
            self.left_canvas.create_text(110, 20, text="Left", font=self.normal_font)
            # 清除右脚画布
            self.right_canvas.delete('all')
            self.right_canvas.create_text(110, 20, text="Right", font=self.normal_font)
        self.master.after(0, update)

    def socket_thread(self):
        # 创建UDP套接字
        local_ip = "127.0.0.1"
        left_port = 53000
        right_port = 53001
        left_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        left_sock.bind((local_ip, left_port))
        right_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        right_sock.bind((local_ip, right_port))

        while True:
            # 接收左脚数据
            data_l, addr_l = left_sock.recvfrom(1024)
            data_l = pickle.loads(data_l)
            sensor_data_l = sensor.parse_sensor_data(data_l)
            pressure_sensors_l = sensor_data_l.pressure_sensors
            current_left_pressure = np.mean(pressure_sensors_l)

            # 接收右脚数据
            data_r, addr_r = right_sock.recvfrom(1024)
            data_r = pickle.loads(data_r)
            sensor_data_r = sensor.parse_sensor_data(data_r)
            pressure_sensors_r = sensor_data_r.pressure_sensors
            current_right_pressure = np.mean(pressure_sensors_r)

            # 当前时间
            current_time = time.time()

            # 检查是否到达绘图间隔
            if current_time - self.last_update_time >= self.update_interval:
                self.last_update_time = current_time  # 更新上一次绘图时间
                self.update_left_canvas(current_left_pressure)
                self.update_right_canvas(current_right_pressure)

                # 阈值录入的状态管理
                if self.recording_left:
                    self.left_pressure_values.append(current_left_pressure)
                    if time.time() - self.recording_start_time >= 3.0:
                        self.recording_left = False
                        avg_pressure = np.mean(self.left_pressure_values)
                        self.thresholds_left[self.current_left_threshold] = avg_pressure
                        self.left_pressure_values = []
                        self.update_status_label(f'Left {self.current_left_threshold} Set')
                elif self.recording_right:
                    self.right_pressure_values.append(current_right_pressure)
                    if time.time() - self.recording_start_time >= 3.0:
                        self.recording_right = False
                        avg_pressure = np.mean(self.right_pressure_values)
                        self.thresholds_right[self.current_right_threshold] = avg_pressure
                        self.right_pressure_values = []
                        self.update_status_label(f'Right {self.current_right_threshold} Set')
                else:
                    if self.thresholds_left or self.thresholds_right:
                        overweight = False
                        overweight_threshold = None
                        # 检查左脚阈值
                        max_exceeded_left = None
                        for Wn, threshold in self.thresholds_left.items():
                            if current_left_pressure > threshold:
                                if max_exceeded_left is None or threshold > self.thresholds_left[max_exceeded_left]:
                                    max_exceeded_left = Wn
                        # 检查右脚阈值
                        max_exceeded_right = None
                        for Wn, threshold in self.thresholds_right.items():
                            if current_right_pressure > threshold:
                                if max_exceeded_right is None or threshold > self.thresholds_right[max_exceeded_right]:
                                    max_exceeded_right = Wn
                        # 判断是否超限
                        if max_exceeded_left or max_exceeded_right:
                            overweight = True
                            exceeded_thresholds = []
                            if max_exceeded_left:
                                exceeded_thresholds.append((self.thresholds_left[max_exceeded_left], f'{max_exceeded_left}'))
                            if max_exceeded_right:
                                exceeded_thresholds.append((self.thresholds_right[max_exceeded_right], f'{max_exceeded_right}'))
                            # 找到最大的超限阈值
                            highest_exceeded = max(exceeded_thresholds, key=lambda x: x[0])
                            overweight_threshold = highest_exceeded[1]
                        self.update_overweight_label(overweight, overweight_threshold)
                        self.update_status_label('Monitoring')
                    else:
                        self.update_status_label('Waiting for Recording')

    def update_status_label(self, text):
        def update():
            self.label_status.config(text=text)
        self.master.after(0, update)

    def update_left_canvas(self, current_pressure):
        def update():
            canvas = self.left_canvas
            canvas.delete('all')  # 清空画布

            # 绘制标题
            canvas.create_text(110, 20, text="Left Foot", font=('Arial', 24))

            # 底部位置
            y0 = 350  # 条形图底部位置
            max_bar_height = 300  # 条形图最大高度

            # 压力范围缩放
            pressure_range = self.pressure_max - self.pressure_min

            # 计算压力比率
            pressure_ratio = (current_pressure - self.pressure_min) / pressure_range
            pressure_ratio = min(max(pressure_ratio, 0), 1)
            bar_height = pressure_ratio * max_bar_height
            y1 = y0 - bar_height

            # 获取压力条颜色
            bar_color = self.get_pressure_color(current_pressure, self.thresholds_left)

            # 绘制压力条
            canvas.create_rectangle(70, y1, 150, y0, fill=bar_color)

            # 绘制阈值线
            for Wn, threshold in self.thresholds_left.items():
                threshold_ratio = (threshold - self.pressure_min) / pressure_range
                threshold_ratio = min(max(threshold_ratio, 0), 1)
                threshold_y = y0 - (threshold_ratio * max_bar_height)
                canvas.create_line(60, threshold_y, 160, threshold_y, fill='blue', dash=(4, 2))
                canvas.create_text(50, threshold_y, text=Wn, anchor='e', font=('Arial', 12))
                # 显示阈值
                canvas.create_text(110, threshold_y - 10, text=f"{threshold:.2f}", font=('Arial', 12))

            # 显示当前压力值
            canvas.create_text(110, y0 + 20, text=f"{current_pressure:.2f}", font=('Arial', 16))
        self.master.after(0, update)

    def update_right_canvas(self, current_pressure):
        def update():
            canvas = self.right_canvas
            canvas.delete('all')  # 清空画布

            # 绘制标题
            canvas.create_text(110, 20, text="Right Foot", font=('Arial', 24))

            # 底部位置
            y0 = 350  # 条形图底部位置
            max_bar_height = 300  # 条形图最大高度

            # 压力范围缩放
            pressure_range = self.pressure_max - self.pressure_min

            # 计算压力比率
            pressure_ratio = (current_pressure - self.pressure_min) / pressure_range
            pressure_ratio = min(max(pressure_ratio, 0), 1)
            bar_height = pressure_ratio * max_bar_height
            y1 = y0 - bar_height

            # 获取压力条颜色
            bar_color = self.get_pressure_color(current_pressure, self.thresholds_right)

            # 绘制压力条
            canvas.create_rectangle(70, y1, 150, y0, fill=bar_color)

            # 绘制阈值线
            for Wn, threshold in self.thresholds_right.items():
                threshold_ratio = (threshold - self.pressure_min) / pressure_range
                threshold_ratio = min(max(threshold_ratio, 0), 1)
                threshold_y = y0 - (threshold_ratio * max_bar_height)
                canvas.create_line(60, threshold_y, 160, threshold_y, fill='blue', dash=(4, 2))
                canvas.create_text(50, threshold_y, text=Wn, anchor='e', font=('Arial', 12))
                # 显示阈值
                canvas.create_text(110, threshold_y - 10, text=f"{threshold:.2f}", font=('Arial', 12))

            # 显示当前压力值
            canvas.create_text(110, y0 + 20, text=f"{current_pressure:.2f}", font=('Arial', 16))
        self.master.after(0, update)

    def get_pressure_color(self, pressure, thresholds):
        """
        根据压力值和阈值计算压力条颜色。
        超过最低阈值时立即变色，然后随压力增加逐渐加深到纯红。
        """
        if not thresholds:
            return '#00FF00'  # 如果没有阈值，默认绿色

        # 获取阈值列表并排序
        sorted_thresholds = sorted(thresholds.values())
        min_threshold = sorted_thresholds[0]
        max_threshold = sorted_thresholds[-1]

        if pressure <= min_threshold:
            # 未超过最低阈值，绿色
            return '#00FF00'
        elif pressure >= max_threshold:
            # 超过最高阈值，纯红
            return '#FF0000'
        else:
            # 超过最低阈值但未达到最高阈值，逐渐从浅红过渡到深红
            # 基础浅红：#FF8080
            base_red = 255
            base_green = 255
            blue = 0

            # 计算加深比例
            ratio = (pressure - min_threshold) / (max_threshold - min_threshold)
            red = base_red
            green = int(base_green * (1 - ratio))  # 绿值逐渐减少

            # 返回颜色值
            return f'#{red:02X}{green:02X}{blue:02X}'


    def update_overweight_label(self, overweight, overweight_threshold=None):
        def update():
            if overweight:
                self.overweight_label.config(
                    text=f"Over Weight {overweight_threshold}!",
                    fg='red',
                    font=self.large_font
                )
            else:
                self.overweight_label.config(
                    text="",
                    fg='black',
                    font=self.large_font
                )
        self.master.after(0, update)

if __name__ == "__main__":
    root = tk.Tk()
    app = MaxDetectionApp(root)
    root.mainloop()
