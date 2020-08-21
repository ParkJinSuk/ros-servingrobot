#! /usr/bin/python
import roslib
import rospy

import tf

from mpu6050 import mpu6050

sensor = mpu6050(0x68)

def update_mpu6050():
    rospy.loginfo("gyro data\t"sensor.get_gyro_data())
    rospy.loginfo("accel data\t"sensor.get_accel_data())

if __name__ == "__main__":
    rospy.init_node('imu_broadcaster')
    while(1):
        # print(sensor.get_gyro_data())
        # print(sensor.get_accel_data())

        rospy.loginfo(sensor.get_gyro_data())
        rospy.sleep(0.1)