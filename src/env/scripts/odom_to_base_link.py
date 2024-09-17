#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

# 回調函數，接收UAV的位置信息
def uav_pose_callback(msg):
    global uav_position, uav_orientation
    # 提取UAV的位置信息 (X, Y, Z)
    uav_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    
    # 提取四元數旋轉
    uav_orientation = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )

def broadcast_odom_to_base_link():
    rospy.init_node('odom_to_base_link_broadcaster')
    br = tf.TransformBroadcaster()

    # 訂閱UAV的位姿數據 (例如AprilTag檢測結果)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, uav_pose_callback)

    rate = rospy.Rate(50)  # 設置頻率為 50 Hz

    # 初始化位置信息
    global uav_position, uav_orientation
    uav_position = (0.0, 0.0, 0.0)
    uav_orientation = (0.0, 0.0, 0.0, 1.0)

    while not rospy.is_shutdown():
        try:
            # 廣播 TF 變換，代表 UAV 在 odom 參考系中的位置和旋轉
            br.sendTransform(
                uav_position,  # UAV 相對於 odom 的位置 (X, Y, Z)
                uav_orientation,  # UAV 相對於 odom 的旋轉 (四元數)
                rospy.Time.now(),
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
