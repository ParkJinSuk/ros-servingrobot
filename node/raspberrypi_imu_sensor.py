#! /usr/bin/python
import roslib
import rospy
import time
import tf

from mpu6050 import mpu6050
from sensor_msgs.msg import Imu

sensor = mpu6050(0x68)

float gyro_x    = 0
float gyro_y    = 0
float gyro_z    = 0
float accel_x   = 0
float accel_y   = 0
float accel_z   = 0

def update_mpu6050():
    gyro_data = sensor.get_gyro_data().split('')
    print(gyro_data[0])
    print(gyro_data[1])
    print(gyro_data[2])
    # rospy.loginfo("gyro data\t" + sensor.get_gyro_data())
    # rospy.loginfo("accel data\t" + sensor.get_accel_data())


# def node_imu_topic():
#     rospy.init_node('imu_broadcaster')

#     while not rospy.is_shutdown():
#         pub.publish()

#         rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        # node_imu_topic()

        while(1):
            update_mpu6050()
            time.sleep(0.5)

    except rospy.ROSInterruptException:
        pass