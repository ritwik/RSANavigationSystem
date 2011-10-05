#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist
from beaconfinder.msg import Location
from beaconfinder.msg import Plan

def callbackPath(data):
	path = data
	
	for point in path.plan:
		Drive(point)
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		if (key == '\x03'):
			break
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def callbackPosition(data):
	global position = data

def control():
	#setting for input for keypress ability
	settings = termios.tcgetattr(sys.stdin)
	
	#need a limit so the robot stops trying to get closer
	global limit = 0.05
	
	rospy.Subscriber("path", Points, callbackPath)
	rospy.Subscriber("position", Position, callbackPosition)
	pubMotion = rospy.Publisher('motion', Motion)
	pubTwist = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('control')
	
	rospy.spin()
	
def Drive(point)
	#finding the distance to the next point
	distance = math.sqrt((position.x-point.x)*(position.x-point.x) + (position.y-point.y)*(position.y-point.y))
	
	if distance < limit
		return

	#finding the turn needed
	angle = atan2((position.y-point.y),(position.x-point.x))
	difference = position.theta - angle
	
	#check difference is too large and readjusts value(i.e. goes over the bound between pi and -pi)
	if abs(difference) > math.pi
		if difference > 0
			difference = difference % pi - pi
		else 
			difference = difference % pi + pi
	
	#determine whether robot needs to go forwards or backwards and then depending on that which way it needs to turn
	if abs(difference) <= math.pi/2
		forward = True
		if difference > 0
			clockwise = True
		else
			clockwise = False
	else
		forward = False
		if difference < 0
			clockwise = True
		else
			clockwise = False
			
	#need to only go part of the distance and then recalculate (possibly call drive recursively)
	'''
	#setting the twist to send to the robot
	twist = Twist()
	twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
	pubTwist.publish(twist)
	'''
	
if __name__ == '__main__':
	control()
