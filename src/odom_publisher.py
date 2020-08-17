#! /usr/bin/env python
import roslib
import rospy
import tf
import geometry_msgs.msg



if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    rospy.loginfo("NODE init [odom_publisher]")

    rospy.Subscriber('/test/pose', geometry_msgs.msg,)