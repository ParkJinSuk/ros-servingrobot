
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ros.h>

#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;

ros::Publisher gyro("/imu", &imu_msg);

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  imu_msg.header.frame_id = 0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;
}

void loop() {
  mpu6050.update();

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;

  imu_msg.angular_velocity.x = mpu6050.getAngleX();
  imu_msg.angular_velocity.y = mpu6050.getAngleY();
  imu_msg.angular_velocity.z = mpu6050.getAngleZ();  

  imu_msg.linear_acceleration.x = mpu6050.getAngleX();
}
