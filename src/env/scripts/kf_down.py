#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Vector3

# quaternion --> rotation matrix
def correct_quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
        [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
        [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
    ])

# 低通濾波器
def low_pass_filter(new_value, prev_value, alpha=0.2):
    return alpha * new_value + (1 - alpha) * prev_value

# 預測下一個狀態
def kf_predict(X0, P0, A, Q, B, U1):
    X10 = np.dot(A, X0) + np.dot(B, U1)
    P10 = np.dot(np.dot(A, P0), A.T) + Q
    return X10, P10

# 測量更新
def kf_update(X10, P10, Z_imu, Z_apriltag, H, R_imu, R_apriltag, alpha):
    # 給 IMU 和 AprilTag 的測量賦予不同權重
    Z_combined = alpha * Z_imu + (1 - alpha) * Z_apriltag
    R_combined = alpha * R_imu + (1 - alpha) * R_apriltag

    K = np.dot(np.dot(P10, H.T), np.linalg.pinv(np.dot(np.dot(H, P10), H.T) + R_combined))
    X1 = X10 + np.dot(K, Z_combined - np.dot(H, X10))
    P1 = np.dot(np.eye(K.shape[0]) - np.dot(K, H), P10)
    return X1, P1, K

# 獲取IMU參數
def imu_cb(data):
    global imu_acceleration
    # 提取IMU quaternion
    quaternion = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ]
    
    # quaternion to rotation matrix
    rotation_matrix = correct_quaternion_to_rotation_matrix(quaternion)

    # 提取IMU加速度值，並將小於0.02的值判定為noise
    edge = 0.02
    linear_acceleration_body = np.array([
        0 if -edge < data.linear_acceleration.x < edge else data.linear_acceleration.x,
        0 if -edge < data.linear_acceleration.y < edge else data.linear_acceleration.y,
        0 if -edge < data.linear_acceleration.z < edge else data.linear_acceleration.z
    ])
    
    # 將IMU加速度值轉換到世界座標系
    linear_acceleration_world = np.dot(rotation_matrix, linear_acceleration_body)
    
    # 考慮重力補償，檢查IMU是否已經進行了重力補償
    gravity = np.array([0, 0, 9.81])
    current_acceleration = linear_acceleration_world - gravity 

    # 使用低通濾波器
    if imu_acceleration is None:
        imu_acceleration = current_acceleration  # 初始值
    else:
        imu_acceleration = low_pass_filter(current_acceleration, imu_acceleration)

def apriltag_measurement_callback(data):
    global apriltag_measurement
    current_measurement = np.array([data.x, data.y, data.z])
    
    # 使用低通濾波器
    if apriltag_measurement is None:
        apriltag_measurement = current_measurement  # 初始值
    else:
        apriltag_measurement = low_pass_filter(current_measurement, apriltag_measurement)

if __name__ == "__main__":
    rospy.init_node("kf_down_node")

    # 卡爾曼濾波器參數
    dt = 1.0 / 50  # 時間步長
    A = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])  # 狀態轉移矩陣

    B = np.array([[0.5 * dt ** 2, 0, 0],
                  [0, 0.5 * dt ** 2, 0],
                  [0, 0, 0.5 * dt ** 2],
                  [dt, 0, 0],
                  [0, dt, 0],
                  [0, 0, dt]])  # 控制輸入矩陣

    H = np.eye(6)[:3, :]  # 測量矩陣 (只測量位置)
    Q = np.eye(6) * 0.01  # 預測噪音(減少系統對噪聲的敏感度)
    R_imu = np.eye(3) * 0.05  # IMU 測量噪音
    R_apriltag = np.eye(3) * 0.2  # AprilTag 測量噪音
    P = np.eye(6) * 0.1  # 初始協方差矩陣
    X = np.zeros(6)  # 初始狀態 [位置x, 位置y, 位置z, 速度x, 速度y, 速度z]
    imu_acceleration = None  # IMU 加速度初始化
    apriltag_measurement = None  # AprilTag 測量初始化

    alpha = 0.2  # 權重，可以調整

    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    rospy.Subscriber("/uav/camera_down_measurement", Vector3, apriltag_measurement_callback)
    position_pub = rospy.Publisher("/kf/camera_down_position", Vector3, queue_size=10)
    rate = rospy.Rate(150)

    while not rospy.is_shutdown():
        if imu_acceleration is not None and apriltag_measurement is not None:
            # 一步預測
            X10, P10 = kf_predict(X, P, A, Q, B, imu_acceleration)

            # 使用結合的 IMU 和 AprilTag 測量更新
            Z_imu = imu_acceleration[:3]  # IMU 的位置
            Z_apriltag = apriltag_measurement  # AprilTag 的位置
            X, P, K = kf_update(X10, P10, Z_imu, Z_apriltag, H, R_imu, R_apriltag, alpha)

            # 發佈估算位置
            pos_msg = Vector3()
            pos_msg.x = X[0]
            pos_msg.y = X[1]
            pos_msg.z = X[2]
            position_pub.publish(pos_msg)

            rospy.loginfo("Predicted camera down position:\nx=%f, y=%f, z=%f", X[0], X[1], X[2])
        else:
            rospy.loginfo("Waiting for IMU and AprilTag data...")

        rate.sleep()
