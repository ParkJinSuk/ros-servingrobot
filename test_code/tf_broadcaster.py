#!/usr/bin/env python  
import roslib
import rospy

import tf
from sensor_msgs.msg import Imu

def handle_imu_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(
                         msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z),
                     rospy.Time.now(),
                     'base_link',
                     "imu_link")

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    
    rospy.Subscriber('/imu', Imu, handle_imu_pose)
    rospy.spin()

'''
def handle_imu_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(
                         msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z),
                     rospy.Time.now(),
                     'base_link',
                     "imu_link")

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    
    rospy.Subscriber('/imu', Imu, handle_imu_pose)
    rospy.spin()
'''