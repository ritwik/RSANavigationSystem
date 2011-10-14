#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist
from beaconfinder.msg import Location
from beaconfinder.msg import Plan
from 

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
	
	#these are the distance and angle travelled in 1.5 second at top speed
	global d = 0.7
	global theta = math.pi/2
	global topSpeed = 0.5
	global topAngular = 1
	
	rospy.Subscriber("path", Points, callbackPath)
	rospy.Subscriber("position", Position, callbackPosition)
	pubMove = rospy.Publisher('move', Move)
	pubTwist = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('control')
	
	rospy.spin()
	
def Drive(point)
	pos = position
	#finding the distance to the next point
	distance = math.sqrt((pos.x-point.x)*(pos.x-point.x) + (pos.y-point.y)*(pos.y-point.y))
	
	if distance < limit
		return

	#finding the turn needed
	angle = atan2((pos.y-point.y),(pos.x-point.x))
	difference = pos.theta - angle
	
	difference = changeAngle(difference, point.isForward)
	
	#setting the twist to send to the robot (first the angle assumes linear which is probably wrong)
	
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = mult*topAngular
	zero = Twist()
	zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
	zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0
	move = Move()
	move.phi = (twist.angular.z/topAngular)*theta
	move.fwd = (twist.linear.x/topSpeed)*d
	
	#does the whole first turn without checking if it is right
	while abs(difference) > theta:
		pubTwist.publish(twist)
		time.sleep(1.5)
		pubTwist.publish(zero)
		pubMove.publish(move)
		if difference < 0
			difference += theta
		else
			difference -= theta
	
	twist.angular.z = topAngular*(theta/difference)
	pubTwist.publish(twist)
	time.sleep(1.5)
	pubTwist.publish(zero)
	move.phi = (twist.angular.z/topAngular)*theta
	pubMove.publish(move)
	
	while distance > limit
		#get the new position
		pos = position
	
		#finding the distance to the next point
		distance = math.sqrt((pos.x-point.x)*(pos.x-point.x) + (pos.y-point.y)*(pos.y-point.y))
	
		#finding the turn needed
		angle = atan2((pos.y-point.y),(pos.x-point.x))
		difference = pos.theta - angle
	
		difference = changeAngle(difference, point.isForward)
		
		if distance > d
			speed = topSpeed
		else
			speed = (distance/d)*topSpeed
		
		if !point.forward
			speed = -speed
		
		if difference > theta
			angularSpeed = topAngular
		else
			angularSpeed = (difference/theta)*topAngular
		
		twist.angular.z = angularSpeed
		twist.linear.x = speed
		
		pubTwist.publish(twist)
		time.sleep(1.5)
		pubTwist.publish(zero)
		move.phi = (twist.angular.z/topAngular)*theta
		move.fwd = (twist.linear.x/topSpeed)*d
		pubMove.publish(move)
	
def changeAngle(angle, forward)
	
	#check difference is too large and readjusts value(i.e. goes over the bound between pi and -pi)
	while angle > math.pi
		angle -= 2*math.pi
	while angle < -math.pi
		angle += 2*math.pi
	
	#determine whether robot needs to go forwards or backwards and then depending on that which way it needs to turn
	if !forward
		if angle < 0
			angle = math.pi + angle
		else
			angle = math.pi - angle
	return angle
	
if __name__ == '__main__':
	control()
