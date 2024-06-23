import numpy as np
import math

g = 9.80665
mag_ref = [0, 0, 0]
dt = 0
I4 = np.eye(4)
I6 = np.eye(6)

def hq(q):
    global g, mag_ref
    q0 = q[0, 0]
    q1 = q[1, 0]
    q2 = q[2, 0]
    q3 = q[3, 0]
    Hx = mag_ref[0]
    Hy = mag_ref[1]
    Hz = mag_ref[2]
    
    h11 = -q2 * g
    h12 = q3 * g
    h13 = -q0 * g
    h14 = q1 * g
    h21 = q1 * g
    h22 = q0 * g
    h23 = q3 * g
    h24 = q2 * g
    h31 = q0 * g
    h32 = -q1 * g
    h33 = -q2 * g
    h34 = q3 * g
    h41 = q0*Hx + q3*Hy - q2*Hz
    h42 = q1*Hx + q2*Hy + q3*Hz
    h43 = -q2*Hx + q1*Hy - q0*Hz
    h44 = -q3*Hx + q0*Hy + q1*Hz
    h51 = -q3*Hx + q0*Hy + q1*Hz
    h52 = q2*Hx - q1*Hy + q0*Hz
    h53 = q1*Hx + q2*Hy + q3*Hz
    h54 = -q0*Hx - q3*Hy + q2*Hz
    h61 = q2*Hx - q1*Hy + q0*Hz
    h62 = q3*Hx - q0*Hy - q1*Hz
    h63 = q0*Hx + q3*Hy - q2*Hz
    h64 = q1*Hx + q2*Hy + q3*Hz
    return 2 * np.array([[h11, h12, h13, h14], [h21, h22, h23, h24], [h31, h32, h33, h34], [h41, h42, h43, h44], [h51, h52, h53, h54], [h61, h62, h63, h64]])

def h(q):
    global g, mag_ref
    q0 = q[0, 0]
    q1 = q[1, 0]
    q2 = q[2, 0]
    q3 = q[3, 0]
    Hx = mag_ref[0]
    Hy = mag_ref[1]
    Hz = mag_ref[2]
    
    h1 = 2 * (q1 * q3 - q0 * q2) * g
    h2 = 2 * (q2 * q3 + q0 * q1) * g
    h3 = (q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2) * g
    h4 = (q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2) * Hx + 2 * (q1 * q2 + q0 * q3) * Hy + 2 * (q1 * q3 - q0 * q2) * Hz
    h5 = 2 * (q1 * q2 - q0 * q3) * Hx + (q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2) * Hy + 2 * (q2 * q3 + q0 * q1) * Hz
    h6 = 2 * (q1 * q3 + q0 * q2) * Hx + 2 * (q2 * q3 - q0 * q1) * Hy + (q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2) * Hz
    return np.array([[h1], [h2], [h3], [h4], [h5], [h6]])

def normalizeQuaternion(q):
    q0 = q[0,0]
    q1 = q[1,0]
    q2 = q[2,0]
    q3 = q[3,0]
    q_norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    return (1 / q_norm) * q

def EKF(gyro, acc, mag, q0, time, Q, R, P, mag_reference):
    global g, mag_ref, dt, I4, I6

    # dt = 1 / fs
    mag_ref = [mag_reference[0], mag_reference[1], mag_reference[2]]

    q_hat_prev = q0
    q = q_hat_prev

    for i in range(len(acc[0]) - 1):
        ax = acc[0][i]
        ay = acc[1][i]
        az = acc[2][i]
        wx = gyro[0][i]
        wy = gyro[1][i]
        wz = gyro[2][i]
        mx = mag[0][i]
        my = mag[1][i]
        mz = mag[2][i]
        dt = time[i+1] - time[i]
        z = np.array([[ax], [ay], [az], [mx], [my], [mz]])

        # Predection
        F = (dt / 2) * np.array([[0, -wx, -wy, -wz], [wx, 0, wz, -wy], [wy, -wz, 0, wx], [wz, wy, -wx, 0]]) + I4
        q_hat_minus = F @ q_hat_prev
        P_minus = F @ P @ F.T + Q
        # Update
        H = hq(q_hat_minus)
        K = P_minus @ H.T @ np.linalg.inv(H @ P_minus @ H.T + R)
        q_hat = q_hat_minus + K @ (z - h(q_hat_minus))
        P = (I4 - K @ H) @ P_minus

        # Next
        q_hat = normalizeQuaternion(q_hat)
        q_hat_prev = q_hat
        q = np.append(q, q_hat_prev, axis=1)

    return q

def quaternion_to_euler(q_list):
    roll = []
    pitch = []
    yaw = []
    q_ = np.transpose(q_list)
    for q in q_:
        # 传入四元数 q 格式：[qw, qx, qy, qz]
        qw, qx, qy, qz = q
        # 横滚角 Roll
        roll.append(np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2)))
        # 俯仰角 Pitch
        pitch.append(np.arcsin(2*(qw*qy - qz*qx)))
        # 偏航角 Yaw
        yaw.append(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))
        
        # 将角度从弧度转换为度
        # roll = np.degrees(roll)
        # pitch = np.degrees(pitch)
        # yaw = np.degrees(yaw)
    
    return roll, pitch, yaw