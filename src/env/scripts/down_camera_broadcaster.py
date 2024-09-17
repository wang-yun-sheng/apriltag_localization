#!/usr/bin/env python3
import rospy
import tf

def broadcast_camera_transform():
    rospy.init_node('down_camera_broadcaster')  # 初始化節點
    br = tf.TransformBroadcaster()  # 創建 TF 廣播器

    rate = rospy.Rate(50)  # 設置頻率為 50 Hz

    while not rospy.is_shutdown():
        # 固定的平移和旋轉
        translation = (0.0, 0.0, -0.03)  # 相機相對於 base_link 的位置 (Z 軸 0.03 米)
        rotation = tf.transformations.quaternion_from_euler(0, 0, 0)  # 無旋轉

        # 廣播相機相對於 base_link 的變換
        br.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "down_camera_link",  # 相機的座標系
            "base_link"  # UAV 的基座標系
        )

        rate.sleep()  # 以 50Hz 的頻率進行廣播

if __name__ == '__main__':
    try:
        broadcast_camera_transform()
    except rospy.ROSInterruptException:
        pass
