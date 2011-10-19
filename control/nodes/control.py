#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy

import math

import pathplanner

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from localisation.msg import State

import sys
import time

#Need a limit so the robot stops trying to get closer (0.1m and 5 degrees) feel free to change
LIMIT = 0.1
ANGULAR_LIMIT = (5 / 180) * math.pi 

#This is the top speed of the robot
TOP_SPEED = 0.5

#This is the distance where the robot begins to slow (0.5m and 45 degrees) they may be wrong, feel free to change them
SLOW_LIMIT = 0.5
ANGULAR_SLOW_LIMIT = math.pi / 4

state = None
settings = None

pubTwist = None

def callbackState(data):
    global state
    state = data

def run():
    print "Waiting for startup keypress"
    line = sys.stdin.readline()

    #Start by spinning around to determine where you are
    spinAround()

    plan = pathplanner.planPath(state)
    for node in plan.path:
        drive(node)

        print "Waiting for next point keypress"
        sys.stdin.readline()


def spinAround():
    #Setting the actual twist to send to the robot0
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.5

    #Set up a zero twist to go between the actual twists
    zero = Twist()
    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

	#The initial turn is done without any linear movement
    pubTwist.publish(zero)
    for i in range(0,4):
        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
        pubTwist.publish(twist)
        time.sleep(1.5)
        
    pubTwist.publish(zero)

def control():
    global pubTwist

    rospy.init_node('control')
    pubTwist = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("State", State, callbackState)

    run()

    rospy.spin()
	
def drive(node):
    global state

    currState = state

    #Finding the distance to the next point
    distance = math.sqrt((currState.x - node.x) ** 2 + (currState.y - node.y) ** 2)
	 
    if distance < LIMIT:
        return

    #HANDLE ROTATION
    #Finding the turn needed
    difference = node.heading - currState.theta
    while difference > math.pi:
        difference -= 2 * math.pi
    while difference < -math.pi:
        difference += 2 * math.pi
	
    #Setting the actual twist to send to the robot0
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 

    #Set up a zero twist to go between the actual twists
    zero = Twist()
    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

	#The initial turn is done without any linear movement
    pubTwist.publish(zero)
    while abs(difference) > ANGULAR_LIMIT:
        #Once it reaches below a certain angle it slows at a ratio of the remaining angle over the limit
        if abs(difference) < ANGULAR_SLOW_LIMIT:
            twist.angular.z = cmp(difference,0) * TOP_SPEED * (difference / ANGULAR_SLOW_LIMIT)
	        
        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
        pubTwist.publish(twist)
        time.sleep(0.1)
        
        #Checks the new difference between current and desired angles
        currState = state
        difference = node.heading - currState.theta
        while difference > math.pi:
            difference -= 2 * math.pi
        while difference < -math.pi:
            difference += 2 * math.pi
    	
    pubTwist.publish(zero)
	
    #HANDLE FORWARDS MOTION
    while distance > LIMIT:
        #Get the new position
        currState = state
	
        #Finding the distance to the next point
        distance = math.sqrt((currState.x - node.x) * (currState.x - node.x) + (currState.y - node.y) * (currState.y - node.y))
	
        #Finding the turn needed
        angle = atan2((currState.y-node.y),(currState.x-node.x))
        difference = angle - currState.theta
        difference = changeAngle(difference, node.forward)
		
        #Slow if within a certain limit of goal
        if distance < SLOW_LIMIT:
            speed = TOP_SPEED * (difference / SLOW_LIMIT)
        else:
            speed = TOP_SPEED
		
        #To make the robot go backwards
        if not node.forward:
            speed = -speed
		
        #Slow if within a certain limit
        if difference < ANGULAR_SLOW_LIMIT:
            angularSpeed = TOP_SPEED * (difference / ANGULAR_SLOW_LIMIT)
        else:
            angularSpeed = TOP_SPEED

        twist.angular.z = angularSpeed
        twist.linear.x = speed

        #Publish the twist and wait a little to recalculate (not sure how long this should be for)		
        pubTwist.publish(twist)
        time.sleep(0.1)

    pubTwist.publish(zero)
	
def changeAngle(angle, forward):
	
    #Check difference is too large and readjusts value(i.e. goes over the bound between pi and -pi)
    while angle > math.pi:
        angle = angle - 2*math.pi
    while angle < -math.pi:
        angle = angle + 2*math.pi
	
    #Determine whether robot needs to go forwards or backwards and then depending on that which way it needs to turn
    if not forward:
        if angle < 0:
            angle = math.pi + angle
        else:
            angle = math.pi - angle
    return angle
	
if __name__ == '__main__':
    control()
