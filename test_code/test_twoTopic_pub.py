#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

if __name__ == "__main__":
    rospy.init_node("test_node_pub")
    pub_twist1 = rospy.Publisher('topic1', Twist, queue_size=50)
    pub_twist2 = rospy.Publisher('topic2', Twist, queue_size=50)

    stamp = 0

    twist1 = Twist()
    twist2 = Twist()
    vx = 0.1
    vy = 0.1
    cnt = 1

    while 1:
        cnt += 1
        rospy.loginfo(cnt)
        twist1.linear.x += 5
        twist2.linear.y += 10
        pub_twist1.publish(twist1)
        pub_twist2.publish(twist2)
        rospy.sleep(0.1)
    pass