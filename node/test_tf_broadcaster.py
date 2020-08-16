#! /usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf
import geometry_msgs.msg



if __name__ == '__main__':
    rospy.init_node('test_tf_broadcaster')
    rospy.Subscriber('/test/pose', geometry_msgs.msg,)