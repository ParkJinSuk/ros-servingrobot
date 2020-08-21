#! /usr/bin/python

from mpu6050 import mpu6050

sensor = mpu6050(0x68)

if __name__ == "__main__":
    
    while(1):
        print(sensor.get_gyro_data())
        print(sensor.get_accel_data())
        pass