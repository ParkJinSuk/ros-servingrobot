#! /usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import tf
import geometry_msgs.msg



if __name__ == '__main__':
    rospy.init_node('robot_state_publisher')
    rospy.loginfo("NODE init [robot_state_publisher]")

    #rospy.Subscriber('/test/pose', geometry_msgs.msg,)