#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

stamp = 0
twist = Twist()
int16 = Int16()

TIME_LIFT_DOWN = 6
TIME_LIFT_UP = 6

def navigation():
    rospy.loginfo("Ready....")
    rospy.sleep(1)
    
    # front 1m
    twist.linear.x = 0.05
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("FRONT 1m")
    rospy.sleep(5) # 5sec

    # stop
    twist.linear.x = 0
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("STOP")
    rospy.sleep(1) # 1sec

    # turn
    twist.linear.x = 0
    twist.angular.z = 0.05
    pub_twist.publish(twist)
    rospy.loginfo("TURN")
    rospy.sleep(10) # 10sec

    # stop
    twist.linear.x = 0
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("STOP")
    rospy.sleep(1) # 1sec

    # front 1m
    twist.linear.x = 0.05
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("FRONT")
    rospy.sleep(5) # 5sec

    # stop
    twist.linear.x = 0
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("STOP")
    rospy.sleep(1) # 1sec

def control_4bar():
    rospy.loginfo("Ready....")
    rospy.sleep(0.5)
    
    # lift down
    int16.data = 1
    pub_4bar.publish(int16)
    rospy.loginfo("lift down {}sec".format(TIME_LIFT_DOWN))
    rospy.sleep(TIME_LIFT_DOWN) # 5.5sec

    # stop
    int16.data = 0
    pub_4bar.publish(int16)
    rospy.loginfo("stop 2sec")
    rospy.sleep(2) # 2sec

    # DC back
    twist.linear.x = -0.07
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("BACK 10sec")
    rospy.sleep(10) # 10sec
    # DC stop
    twist.linear.x = 0
    twist.angular.z = 0
    pub_twist.publish(twist)
    rospy.loginfo("stop 2sec")
    rospy.sleep(2) # 2sec

    # lift up
    int16.data = 2
    pub_4bar.publish(int16)
    rospy.loginfo("lift up 6sec")
    rospy.sleep(TIME_LIFT_UP) # 6sec

    # stop
    int16.data = 0
    pub_4bar.publish(int16)
    rospy.loginfo("stop 1sec")
    rospy.sleep(1) # 1sec


if __name__ == "__main__":
    rospy.init_node("node_sena_cmd_pub")
    pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=50)
    pub_4bar = rospy.Publisher('cmd_vel_4bar', Int16, queue_size=50)

    # navigation()

    control_4bar()
    