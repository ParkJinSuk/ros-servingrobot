#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage



if __name__ == "__main__":
    rospy.init_node("test_node_imu_pub")
    pub_imu = rospy.Publisher('imu', Imu, queue_size=50)

    stamp = 0

    imu = Imu()
    imu.header.seq = 0
    
    while 1:
        stamp += 1
        imu.header.seq = stamp
        pub_imu.publish(imu)
        rospy.sleep(0.1)
    pass