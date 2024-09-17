#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry

# quaternion --> rotation matrix
def correct_quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
        [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
        [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
    ])

def apriltag(x, y, z):
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    return p

apriltags = [apriltag(0, 0, 0),apriltag(-0.4, 0, 0),apriltag(0.4, 0, 0),apriltag(0, 0.4, 0),apriltag(0, -0.4, 0),apriltag(4.5, 0, 2.5)]

# Callback for tag detections
# Callback for tag detections
def tag_detections_callback(msg, pub):
    for detection in msg.detections:
        # 確保我們處理的 id 是整數
        if isinstance(detection.id, (list, tuple)) and len(detection.id) > 0:
            tag_id = detection.id[0]
        else:
            continue  # 如果 ID 結構不對，跳過這個檢測

        # 檢查是否在有效範圍內
        if tag_id < len(apriltags):
            try:
                # 根據 PoseWithCovariance 的結構提取位置和方向
                pose_with_covariance = detection.pose.pose
                position = pose_with_covariance.pose.position  # 修改此行
                orientation = pose_with_covariance.pose.orientation  # 修改此行

                rel_pos_b = np.array([position.x, position.y, position.z])
                q = [orientation.x, orientation.y, orientation.z, orientation.w]
                R = correct_quaternion_to_rotation_matrix(q)

                rel_pos = np.dot(R, rel_pos_b)

                tag_pos = np.array([
                    apriltags[tag_id].pose.position.x,
                    apriltags[tag_id].pose.position.y,
                    apriltags[tag_id].pose.position.z
                ])

                measurement = tag_pos - rel_pos

                measurement_msg = Vector3()
                measurement_msg.x = measurement[0]
                measurement_msg.y = measurement[1]
                measurement_msg.z = measurement[2]
                pub.publish(measurement_msg)
            except AttributeError as e:
                rospy.logerr(f"AttributeError: {e}. Pose data structure might be different.")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")


# Callback for local position
def local_position_callback(data, pub):
    local_pos_msg = Vector3()
    local_pos_msg.x = data.pose.pose.position.x  # 從 Odometry 中提取位置
    local_pos_msg.y = data.pose.pose.position.y
    local_pos_msg.z = data.pose.pose.position.z
    pub.publish(local_pos_msg)

# Main function
def main():
    rospy.init_node('apriltag_camera_down_measure_uav')

    measurement_pub = rospy.Publisher('/uav/camera_down_measurement', Vector3, queue_size=10)
    local_position_pub = rospy.Publisher('/uav/camera_down_local_position', Vector3, queue_size=10)

    rospy.Subscriber('/iris/truth_pose', Odometry, local_position_callback, local_position_pub)
    rospy.Subscriber('/camera_down/tag_detections', AprilTagDetectionArray, tag_detections_callback, measurement_pub)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass