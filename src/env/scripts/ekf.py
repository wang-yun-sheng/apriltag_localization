#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def odometry_callback(data):
    # 這裡可以對接收到的數據進行處理
    rospy.loginfo("Received odometry: %s", data)

    # 發佈到其他主題
    odom_pub.publish(data)

def main():
    rospy.init_node('ekf')

    global odom_pub
    odom_pub = rospy.Publisher('/odometry/forwarded', Odometry, queue_size=10)

    rospy.Subscriber('', Odometry, odometry_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
