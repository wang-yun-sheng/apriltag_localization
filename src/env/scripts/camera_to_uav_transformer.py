#!/usr/bin/env python
import rospy
import tf
import numpy as np

def transform_camera_to_uav():
    rospy.init_node('camera_to_uav_transformer')
    listener = tf.TransformListener()

    rate = rospy.Rate(50)  # 設置頻率為 50 Hz

    while not rospy.is_shutdown():
        try:
            # 等待 TF 變換
            listener.waitForTransform("base_link", "down_camera_link", rospy.Time(0), rospy.Duration(3.0))
            
            # 查詢變換
            (trans, rot) = listener.lookupTransform("base_link", "down_camera_link", rospy.Time(0))
            
            # 將四元數轉換為旋轉矩陣
            x, y, z, w = rot
            rotation_matrix = np.array([
                [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
            ])
            
            local_point = np.array([0, 0, 0.03])  # 假設某個點在 down_camera_link 座標系中的坐標
            transformed_point = np.dot(rotation_matrix, local_point) + np.array(trans)
            
            # transformed_point 現在包含從 down_camera_link 轉換到 base_link 的座標
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

if __name__ == '__main__':
    try:
        transform_camera_to_uav()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
import tf
import numpy as np

def transform_camera_to_uav():
    rospy.init_node('camera_to_uav_transformer')
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 設置頻率為 50 Hz

    while not rospy.is_shutdown():
        try:
            # 等待 TF 變換
            listener.waitForTransform("base_link", "down_camera_link", rospy.Time(0), rospy.Duration(3.0))
            
            # 查詢變換
            (trans, rot) = listener.lookupTransform("base_link", "down_camera_link", rospy.Time(0))
            
            # 將四元數轉換為旋轉矩陣
            x, y, z, w = rot
            rotation_matrix = np.array([
                [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
            ])
            
            local_point = np.array([0, 0, 0.03])  # 假設某個點在 down_camera_link 座標系中的坐標
            transformed_point = np.dot(rotation_matrix, local_point) + np.array(trans)
            
            # transformed_point 現在包含從 down_camera_link 轉換到 base_link 的座標
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

if __name__ == '__main__':
    try:
        transform_camera_to_uav()
    except rospy.ROSInterruptException:
        pass
