#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage



if __name__ == "__main__":
    rospy.init_node("test_node_enc_pub")
    pub_twist = rospy.Publisher('encoder_vel', Twist, queue_size=50)

    stamp = 0

    twist = Twist()
    vx = 0.1
    vy = 0.1
    cnt = 1

    while 1:
        cnt += 1
        rospy.loginfo(cnt)
        twist.angular.x = 5
        twist.angular.y = 10
        pub_twist.publish(twist)
        rospy.sleep(0.1)
    pass