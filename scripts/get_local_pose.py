#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
import sys
from gazebo_msgs.msg import ModelStates
import tf2_ros

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# 只发布单个无人机数据到 /mavros 话题（通常是第一个）
pose_pub = None
odom_pub = None
speed_pub = None
local_pose = PoseStamped()
odom = Odometry()
speed = Vector3Stamped()

# TF 广播器
tf_broadcaster = None

def gazebo_model_state_callback(msg):
    global local_pose, odom, speed, tf_broadcaster
    
    try:
        # 查找第一个匹配的 vehicle（vehicle_id=0）
        vehicle_name = vehicle_type + '_0'
        id = msg.name.index(vehicle_name)
        
        current_time = rospy.Time.now()
        
        # 更新 pose
        local_pose.header.stamp = current_time
        local_pose.header.frame_id = 'map'
        local_pose.pose = msg.pose[id]
        
        # 更新 odometry
        odom.header.stamp = current_time
        odom.header.frame_id = 'map'
        odom.child_frame_id = "body"
        odom.pose.pose = msg.pose[id]
        odom.twist.twist = msg.twist[id]
        
        # 更新 speed
        speed.header.stamp = current_time
        speed.header.frame_id = 'map'
        speed.vector = msg.twist[id].linear
        
        # ========== 广播 TF 变换 ==========
        # 广播 map -> body (机体坐标系)
        tf_map_to_base = TransformStamped()
        tf_map_to_base.header.stamp = current_time
        tf_map_to_base.header.frame_id = "map"
        tf_map_to_base.child_frame_id = "body"
        tf_map_to_base.transform.translation.x = msg.pose[id].position.x
        tf_map_to_base.transform.translation.y = msg.pose[id].position.y
        tf_map_to_base.transform.translation.z = msg.pose[id].position.z
        tf_map_to_base.transform.rotation = msg.pose[id].orientation
        
        # 发布 TF
        tf_broadcaster.sendTransform(tf_map_to_base)
        
    except ValueError:
        rospy.logwarn_throttle(5, f"Vehicle '{vehicle_name}' not found in Gazebo model states")

if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')
    
    # 初始化 TF 广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # 订阅 Gazebo 模型状态
    gazebo_model_state_sub = rospy.Subscriber(
        "/gazebo/model_states", 
        ModelStates, 
        gazebo_model_state_callback,
        queue_size=100
    )
    
    # 发布到 /mavros 话题（不带 vehicle_type 前缀）
    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    odom_pub = rospy.Publisher('/mavros/local_position/odom', Odometry, queue_size=1)
    speed_pub = rospy.Publisher('/mavros/vision_speed/speed', Vector3Stamped, queue_size=1)
    
    rospy.loginfo(f"Publishing {vehicle_type}_0 groundtruth to /mavros topics")
    rospy.loginfo("Topics:")
    rospy.loginfo("  - /mavros/vision_pose/pose")
    rospy.loginfo("  - /mavros/local_position/odom")
    rospy.loginfo("  - /mavros/vision_speed/speed")
    rospy.loginfo("")
    rospy.loginfo("TF Broadcasting:")
    rospy.loginfo("  - map -> body (Body/IMU frame)")
    rospy.loginfo("")
    rospy.loginfo("Note: Camera points are published in body frame")
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        pose_pub.publish(local_pose)
        odom_pub.publish(odom)
        speed_pub.publish(speed)
        try:
            rate.sleep()
        except:
            continue

