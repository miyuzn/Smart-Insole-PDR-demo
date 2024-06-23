import numpy as np

def calculate_shoe_detector(accelerometer_data, gyroscope_data, gravity=9.81, sigma_a=0.02, sigma_w=0.1, gamma=1.0):
    """
    Implement the SHOE detector based on GLRT for zero-velocity detection in IMU data.
    
    :param accelerometer_data: N x 3 numpy array of accelerometer readings (m/s^2)
    :param gyroscope_data: N x 3 numpy array of gyroscope readings (rad/s)
    :param gravity: magnitude of gravity (m/s^2)
    :param sigma_a: standard deviation of accelerometer noise
    :param sigma_w: standard deviation of gyroscope noise
    :param gamma: threshold for detection decision
    :return: Boolean array indicating zero-velocity detections
    """
    N = accelerometer_data.shape[0]
    
    # Calculate the mean specific force vector
    ya_n = np.mean(accelerometer_data, axis=0)
    ya_n_norm = np.linalg.norm(ya_n)
    
    # Normalize the mean specific force vector to get the unit vector in the direction of gravity
    u_hat = ya_n / ya_n_norm
    
    # Calculate T(z_n) for the GLRT
    T_zn = (1 / (N * sigma_a ** 2)) * np.sum([
        np.linalg.norm(accelerometer_data[i] - gravity * u_hat) ** 2 +
        (1 / sigma_w ** 2) * np.linalg.norm(gyroscope_data[i]) ** 2
        for i in range(N)
    ])
    
    # Decision rule: if T_zn is less than gamma, the detector decides the IMU is stationary
    if T_zn < gamma:
        return 0
    else:
        return 1

# Example usage:
# Assuming `accel_data` and `gyro_data` are loaded as Nx3 matrices representing sensor readings
# Example call: detect_stationary = calculate_shoe_detector(accel_data, gyro_data)
def shoe(acc_data, gyro_data, gravity=9.81, sigma_a=0.02, sigma_w=0.1, gamma=1.0):
    L = []
    for i in range(len(acc_data[0])):
        L.append(calculate_shoe_detector(acc_data[i], gyro_data[i], gravity, sigma_a, sigma_w, gamma))
    return L