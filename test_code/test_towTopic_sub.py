#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

def callback_imu(data):
    rospy.loginfo("imu callback")
    rospy.loginfo(data)
    pass

def callback_tf(data):
    rospy.loginfo(data)
    pass

if __name__ == "__main__":
    rospy.init_node("test_node_twoTopic_sub")
    sub_tf = rospy.Subscriber('tf', TFMessage, callback_tf, queue_size=50)
    sub_imu = rospy.Subscriber('imu', Imu, callback_imu, queue_size=50)
    rospy.spin()
    pass