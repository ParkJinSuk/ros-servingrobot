#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


msg = """
Control Your ServingRobot! 4Bar
---------------------------
Moving around:

q : 4Bar lift down
w : 4Bar Stop && ServingRobot back
e : 4Bar lift up
s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -ROBOT_MAX_LIN_VEL, ROBOT_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -ROBOT_MAX_ANG_VEL, ROBOT_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('servingrobot_4bar_control')
    pub = rospy.Publisher('cmd_vel_4bar', String, queue_size=50)

    status = 0
    # target_linear_vel   = 0.0
    # target_angular_vel  = 0.0
    # control_linear_vel  = 0.0
    # control_angular_vel = 0.0

    try:
        string = String()
        print(msg)
        while(1):
            key = getKey()
            if key == 'q' :
                string.data = "q"
                status = status + 1
                print(string.data)
            elif key == 'w' :
                string.data = "w"
                status = status + 1
                print(string.data)
            elif key == 'e' :
                string.data = "e"
                status = status + 1
                print(string.data)
            elif key == 's' :
                string.data = "s"
                status = status + 1
                print(string.data)
            elif key == ' ' or key == 's' :
                pass
            else:
                if (key == '\x03'):
                    break

            if status == 10 :
                print(msg)
                status = 0

            pub.publish(string)

    except:
        print(e)

    finally:
        string = String()
        string.data = 's'
        pub.publish(string)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
