#!/usr/bin/env python
import rospy
import pymysql
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# pymysql
my_db = pymysql.connect(
    user='sexymandoo',
    passwd='sexymandoo',
    host='101.101.218.239',
    db='project',
    charset='utf8'
)
cursor = my_db.cursor(pymysql.cursors.DictCursor)
sql_all = "SELECT * FROM `orderDB`;"
sql_callArduino = "SELECT _table FROM project.orderDB WHERE call_arduino = '1' AND serving = '0';"
sql_completecall = "UPDATE orderDB SET serving = '1' WHERE call_arduino = '1' AND serving = '0';"
# mypysql

stamp = 0
twist = Twist()
int16 = Int16()

TIME_LIFT_DOWN = 6
TIME_LIFT_UP = 6

# navigation
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

# 4bar
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
    # control_4bar()
    
    try:
        while True:
            rospy.sleep(1)
            my_db.commit()
            cursor.execute(sql_callArduino)
            call_table_list = cursor.fetchall()
            print("Call num : {}".format(len(call_table_list)))
            
            if(len(call_table_list) > 0):
                print("\tserving start!")
                table_number = call_table_list[0]
                print("table_number {}".format(table_number))

                control_4bar()
                
                cursor.execute(sql_completecall)
                print("ehltsi?$$$$$$$$$$$$$$$$$")
                my_db.commit()
                print("\tserving end!")
            
            # order_table_num = call_table_list[0]['_table']

    except KeyboardInterrupt:
        my_db.close()
    