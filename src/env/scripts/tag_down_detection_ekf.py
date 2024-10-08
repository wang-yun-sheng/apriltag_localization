#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, PoseStamped
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

apriltags = [
    apriltag(0, 0, 0), 
    apriltag(0.3, 0, 0), 
    apriltag(0.6, 0, 0), 
    apriltag(0.9, 0, 0), 
    apriltag(1.2, 0, 0), 
    apriltag(-0.3, 0, 0), 
    apriltag(-0.6, 0, 0), 
    apriltag(-0.9, 0, 0), 
    apriltag(-1.2, 0, 0), 
    
]

def tag_detections_callback(msg, pub):
    for detection in msg.detections:
        if isinstance(detection.id, (list, tuple)) and len(detection.id) > 0:
            tag_id = detection.id[0]
        else:
            continue

        if tag_id < len(apriltags):
            try:
                # 使用正確的屬性路徑訪問數據
                pose_with_covariance = detection.pose

                # 確認 pose_with_covariance 的結構
                if hasattr(pose_with_covariance, 'pose') and hasattr(pose_with_covariance.pose, 'pose'):
                    position = pose_with_covariance.pose.pose.position  # 修正访问路径
                    orientation = pose_with_covariance.pose.pose.orientation          
  
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

                    measurement_msg = PoseWithCovarianceStamped()
                    measurement_msg.header.stamp = rospy.Time.now()
                    measurement_msg.header.frame_id = "odom"
                    measurement_msg.pose.pose.position.x = measurement[0]
                    measurement_msg.pose.pose.position.y = measurement[1]
                    measurement_msg.pose.pose.position.z = measurement[2]
                    measurement_msg.pose.covariance = [0.001] * 36

                    pub.publish(measurement_msg)
                else:
                    rospy.logerr(f"PoseWithCovariance structure is not as expected. Detection: {detection}")

            except AttributeError as e:
                rospy.logerr(f"AttributeError: {e}. Pose data structure might be different. Detection: {detection}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

# Callback for local position
def local_position_callback(data, pub):
    try:
        local_pos_msg = Vector3()
        local_pos_msg.x = data.pose.position.x  # 從 Odometry 中提取位置，直接访问
        local_pos_msg.y = data.pose.position.y
        local_pos_msg.z = data.pose.position.z
        pub.publish(local_pos_msg)
    except AttributeError as e:
        rospy.logerr(f"AttributeError in local_position_callback: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error in local_position_callback: {e}")

# Main function
def main():
    rospy.init_node('apriltag_camera_down_measure_uav_ekf')

    measurement_pub = rospy.Publisher('/uav/camera_down_measurement_ekf', PoseWithCovarianceStamped, queue_size=20)
    local_position_pub = rospy.Publisher('/uav/camera_down_local_position_ekf', Vector3, queue_size=10)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_position_callback, local_position_pub)
    rospy.Subscriber('/camera_down/tag_detections', AprilTagDetectionArray, tag_detections_callback, measurement_pub)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
