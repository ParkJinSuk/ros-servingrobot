#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("node_sena_cmd_pub")
    pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=50)

    stamp = 0

    twist = Twist()
    
    # front 1m
    twist.linear.x = 0.05
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("FRONT 1m")
    rospy.sleep(20) # 20sec

    # stop
    twist.linear.x = 0
    twist.linear.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("STOP")
    rospy.sleep(1) # 1sec

    # turn
    twist.linear.x = 0
    twist.linear.z = 0.02
    pub_twist.publish(twist)
    rospy.loginfo("TURN")
    rospy.sleep(10) # 10sec

    # front 1m
    twist.linear.x = 0.05
    twist.linear.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("FRONT")
    rospy.sleep(20) # 20sec