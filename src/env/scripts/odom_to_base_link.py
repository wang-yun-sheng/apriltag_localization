#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 初始化全局变量
uav_position = (0.0, 0.0, 0.0)
uav_orientation = (0.0, 0.0, 0.0, 1.0)
last_timestamp = None

# 回調函數，接收UAV的位置信息
def uav_pose_callback(msg):
    global uav_position, uav_orientation, last_timestamp

    # 提取UAV的位置信息 (X, Y, Z)
    uav_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    
    # 提取四元數旋轉
    uav_orientation = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    
    # 更新时间戳
    last_timestamp = msg.header.stamp

def broadcast_odom_to_base_link():
    global last_timestamp
    
    rospy.init_node('odom_to_base_link_broadcaster')
    br = tf.TransformBroadcaster()

    # 訂閱UAV的位姿數據 (例如AprilTag檢測結果)
    rospy.Subscriber('/odometry/filtered', Odometry, uav_pose_callback)

    rate = rospy.Rate(50)  # 設置頻率為 50 Hz

    while not rospy.is_shutdown():
        try:
            # 检查当前的时间戳是否与上次相同
            if last_timestamp is not None:
                br.sendTransform(
                    uav_position,  # UAV 相對於 odom 的位置 (X, Y, Z)
                    uav_orientation,  # UAV 相對於 odom 的旋轉 (四元數)
                    last_timestamp,  # 使用最新的时间戳
                    "base_link",  # UAV 本身的座標系
                    "odom"  # 參考的odom世界座標系
                )
        except tf.Exception as e:
            rospy.logerr("TF Exception: %s", str(e))
        except rospy.ROSInterruptException:
            rospy.loginfo("Node interrupted, shutting down.")
        except Exception as e:
            rospy.logerr("Unexpected exception: %s", str(e))

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_odom_to_base_link()
    except rospy.ROSInterruptException:
        pass
