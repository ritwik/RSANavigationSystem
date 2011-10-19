#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def fwd(x):
  twist = Twist()
  twist.linear.x = -x; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # th*turn
  pub.publish(twist)
  time.sleep(1.5)
  twist = Twist()
  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # th*turn
  pub.publish(twist)

def turn(x):
  twist = Twist()
  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = x # th*turn
  pub.publish(twist)
  time.sleep(1.5)
  twist = Twist()
  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 # th*turn
  pub.publish(twist)


if __name__=="__main__":
  pub = rospy.Publisher('cmd_vel', Twist)
  rospy.init_node('my_twist_keyboard')
  for i in range(4):
    fwd(0.5)
    turn(1)
  print "YAY"
