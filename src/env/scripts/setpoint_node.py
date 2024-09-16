#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def Pose_cb(msg):
    global current_pose
    current_pose = msg

def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 )

# 定義航點
def Waypoint(x, y, z):
    set_point = PoseStamped()
    set_point.pose.position.x = x
    set_point.pose.position.y = y
    set_point.pose.position.z = z
    return set_point

waypoints = [
    Waypoint(0, 0, 2),
    Waypoint(1, 1, 2),
    Waypoint(-1, 1, 2),
    Waypoint(1, -1, 2),
    Waypoint(-1, -1, 2),
]

if __name__ == "__main__":
    rospy.init_node("setpoint_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, Pose_cb)

    local_setpoint_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(50)

    # 等待飛行控制器連線
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # 在開始前發送一些 setpoints
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        local_setpoint_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    waypoint_index = 0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD 已啟用")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("飛行器已解鎖")
            
                last_req = rospy.Time.now()

        # 檢查無人機是否到達當前航點
        if distance(current_pose.pose.position, waypoints[waypoint_index].pose.position) < 0.25 :
            waypoint_index += 1
            if waypoint_index >= len(waypoints):
                waypoint_index = 0

        # 設定當前航點的姿態
        pose.pose.position.x = waypoints[waypoint_index].pose.position.x
        pose.pose.position.y = waypoints[waypoint_index].pose.position.y
        pose.pose.position.z = waypoints[waypoint_index].pose.position.z
        
        local_setpoint_pos_pub.publish(pose)

        rate.sleep()
