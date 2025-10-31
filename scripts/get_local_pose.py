#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TransformStamped
from nav_msgs.msg import Odometry
import sys
from gazebo_msgs.msg import ModelStates
import tf2_ros
import threading

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# 发布器
pose_pub = None
odom_pub = None
speed_pub = None

# TF 广播器
tf_broadcaster = None

# 存储最新的位姿数据（由订阅线程更新，由定时器线程读取）
latest_pose = None
latest_twist = None
data_lock = threading.Lock()  # 保护共享数据
data_received = False

# 上一次发布的时间戳（用于去重，防御性编程）
last_published_time = None


def gazebo_model_state_callback(msg):
    """订阅 Gazebo 状态，只负责更新数据，不发布"""
    global latest_pose, latest_twist, data_received

    try:
        # 查找第一个匹配的 vehicle（vehicle_id=0）
        vehicle_name = vehicle_type + '_0'
        id = msg.name.index(vehicle_name)

        # 使用线程锁保护共享数据
        with data_lock:
            latest_pose = msg.pose[id]
            latest_twist = msg.twist[id]
            data_received = True

    except ValueError:
        rospy.logwarn_throttle(
            5, f"Vehicle '{vehicle_name}' not found in Gazebo model states")


def publish_timer_callback(event):
    """定时器回调：负责定期发布 TF 和话题"""
    global latest_pose, latest_twist, data_received, last_published_time

    # 检查是否收到数据
    with data_lock:
        if not data_received:
            return
        pose = latest_pose
        twist = latest_twist

    current_time = rospy.Time.now()

    # 时间戳去重：防止对同一时间戳重复发布 TF（防御性编程）
    if last_published_time is not None and current_time == last_published_time:
        return
    last_published_time = current_time

    # 发布 PoseStamped
    local_pose = PoseStamped()
    local_pose.header.stamp = current_time
    local_pose.header.frame_id = 'map'
    local_pose.pose = pose
    pose_pub.publish(local_pose)

    # 发布 Odometry
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'map'
    odom.child_frame_id = "base_link"
    odom.pose.pose = pose
    odom.twist.twist = twist
    odom_pub.publish(odom)

    # 发布 Speed
    speed = Vector3Stamped()
    speed.header.stamp = current_time
    speed.header.frame_id = 'map'
    speed.vector = twist.linear
    speed_pub.publish(speed)

    # 广播 TF: map -> body
    tf_map_to_body = TransformStamped()
    tf_map_to_body.header.stamp = current_time
    tf_map_to_body.header.frame_id = "map"
    tf_map_to_body.child_frame_id = "base_link"
    tf_map_to_body.transform.translation.x = pose.position.x
    tf_map_to_body.transform.translation.y = pose.position.y
    tf_map_to_body.transform.translation.z = pose.position.z
    tf_map_to_body.transform.rotation = pose.orientation
    tf_broadcaster.sendTransform(tf_map_to_body)


if __name__ == '__main__':
    rospy.init_node('get_pose_groundtruth')

    # 初始化 TF 广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 发布到 /mavros 话题（不带 vehicle_type 前缀）
    pose_pub = rospy.Publisher(
        '/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    odom_pub = rospy.Publisher(
        '/mavros/local_position/odom', Odometry, queue_size=10)
    speed_pub = rospy.Publisher(
        '/mavros/vision_speed/speed', Vector3Stamped, queue_size=10)

    # 订阅 Gazebo 模型状态（queue_size=1，只保留最新数据）
    gazebo_model_state_sub = rospy.Subscriber(
        "/gazebo/model_states",
        ModelStates,
        gazebo_model_state_callback,
        queue_size=1
    )

    # 创建定时器：100Hz 定时发布 TF 和话题
    publish_rate = 150  # Hz
    timer = rospy.Timer(rospy.Duration(1.0 / publish_rate),
                        publish_timer_callback)

    rospy.loginfo(f"Publishing {vehicle_type}_0 groundtruth to /mavros topics")
    rospy.loginfo("Architecture:")
    rospy.loginfo(
        "  - Subscriber thread: updates latest pose from Gazebo (1000Hz)")
    rospy.loginfo(
        f"  - Timer thread: publishes TF and topics at {publish_rate}Hz")
    rospy.loginfo("")
    rospy.loginfo("Topics:")
    rospy.loginfo("  - /mavros/vision_pose/pose")
    rospy.loginfo("  - /mavros/local_position/odom")
    rospy.loginfo("  - /mavros/vision_speed/speed")
    rospy.loginfo("")
    rospy.loginfo("TF Broadcasting:")
    rospy.loginfo("  - map -> body (Body/IMU frame)")
    rospy.loginfo("")
    rospy.loginfo(
        "Note: This avoids TF_REPEATED_DATA warnings by rate-limiting TF broadcasts")

    # 保持节点运行
    rospy.spin()
