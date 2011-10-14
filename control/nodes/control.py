#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy

import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from localisation.msg import State
from pathplanner.msg import Path, PathNode

#need a limit so the robot stops trying to get closer
LIMIT = 0.05

#these are the distance and angle travelled in 1.5 second at top speed
D = 0.7
THETA = math.pi/2
TOP_SPEED = 0.5

state = None

def callbackPath(data):
	path = data
	
	for node in path.nodes:
		Drive(node)

        #Wait for keypress
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		if (key == '\x03'):
			break
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def callbackPosition(data):
	global state
    state = data

def control():
	#setting for input for keypress ability
	settings = termios.tcgetattr(sys.stdin)
	

	rospy.Subscriber("path", Points, callbackPath)
	rospy.Subscriber("position", Position, callbackPosition)
	pubMove = rospy.Publisher('move', Move)
	pubTwist = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('control')
	
	rospy.spin()
	
def Drive(node)
    while (state == None):
        print "Does not have state yet"
	currState = state

	#Finding the distance to the next point
	distance = math.sqrt((currState.x - point.x) ** 2 + (currState.y - point.y) ** 2)
	 
	if distance < limit
		return

    #HANDLE ROTATION
	#Finding the turn needed
    difference = node.heading - currState.theta
    while difference > math.pi
		difference -= 2*math.pi
	while difference < -math.pi
		difference += 2*math.pi
	
	#Setting the actual twist to send to the robot
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = cmp(number,0) * TOP_SPEED

    #Set up a zero twist to go between the actual twists
	zero = Twist()
	zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
	zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

    #Set up the odometry to send for every twist
	move = Move()
	move.phi = (twist.angular.z / TOP_SPEED) * THETA
	move.fwd = (twist.linear.x / TOP_SPEED) * D


    #Motion updates are going to be based on odometry instead
    #Then using our observations, we detect when we're in the correct state and stop
        #But it's not exactly stop
        #It's more like, as you get closer, slow down
	
	#Does the whole first turn without checking if it is right
    pubTwist.publish(zero)
	while abs(difference) > THETA:    
		pubTwist.publish(twist)
		time.sleep(1.5)
		pubTwist.publish(zero)
		pubMove.publish(move)

        #Checks the new difference between current and desired angles
        difference = node.heading - currState.theta
        while difference > math.pi
    		difference -= 2*math.pi
    	while difference < -math.pi
    		difference += 2*math.pi
    	
    #Reduce velocity as difference gets smaller (assumes linearity)
	twist.angular.z = topAngular * (theta / difference)
	pubTwist.publish(twist)
	time.sleep(1.5)
	pubTwist.publish(zero)
	move.phi = (twist.angular.z / topAngular) * theta
	pubMove.publish(move)
	
    #HANDLE FORWARDS MOTION
	while distance > limit
		#Get the new position
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
