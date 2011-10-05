#!/usr/bin/env python
import roslib; roslib.load_manifest('beaconfinder')
import rospy
import time
from geometry_msgs.msg import Twist

import sys, select, termios, tty

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('cmd_vel', Twist)
  rospy.init_node('my_twist_keyboard')
  while 1:
    line = 0
    try:
        line = sys.stdin.readline()
    except IOError:
        end = time.time()
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        print(end-start)
        break

    if line.strip() == '':
        end = time.time()
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        print(end-start)
        break

    x = int(line.strip())
    twist = Twist()
    twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # th*turn
    pub.publish(twist)
    start = time.time()

