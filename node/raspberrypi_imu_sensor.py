#! /usr/bin/python
import roslib
import rospy

import tf

from mpu6050 import mpu6050
from sensor_msgs.msg import Imu

sensor = mpu6050(0x68)

float gyro_x
float gyro_y
float gyro_z
float accel_x
float accel_y
float accel_z

def update_mpu6050():
    gyro_data = sensor.get_gyro_data().split()
    rospy.loginfo(gyro_data[0])
    # rospy.loginfo("gyro data\t" + sensor.get_gyro_data())
    # rospy.loginfo("accel data\t" + sensor.get_accel_data())

def node_imu_topic():
    rospy.init_node('imu_broadcaster')

    while not rospy.is_shutdown():
        pub.publish()

        rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        node_imu_topic()
    except rospy.ROSInterruptException:
        pass