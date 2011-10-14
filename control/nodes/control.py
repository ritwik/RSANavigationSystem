#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy

import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from localisation.msg import State
from pathplanner.msg import Path, PathNode

#Need a limit so the robot stops trying to get closer (0.1m and 5 degrees) feel free to change
LIMIT = 0.1
ANGULAR_LIMIT = (5 / 180) * math.pi 

#This is the top speed of the robot
TOP_SPEED = 0.5

#This is the distance where the robot begins to slow (0.5m and 45 degrees) they may be wrong, feel free to change them
SLOW_LIMIT = 0.5
ANGULAR_SLOW_LIMIT = math.pi / 4

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

def callbackState(data):
    global state
    state = data

def control():
    #Setting for input for keypress ability
    settings = termios.tcgetattr(sys.stdin)
	
    rospy.Subscriber("path", Path, callbackPath)
    rospy.Subscriber("state", State, callbackState)
    pubTwist = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('control')
	
    rospy.spin()
	
def Drive(node)
    while (state == None):
        print "Does not have state yet"
    currState = state

    #Finding the distance to the next point
    distance = math.sqrt((currState.x - node.x) ** 2 + (currState.y - node.y) ** 2)
	 
    if distance < LIMIT
        return

    #HANDLE ROTATION
    #Finding the turn needed
    difference = node.heading - currState.theta
    while difference > math.pi
        difference -= 2*math.pi
    while difference < -math.pi
        difference += 2*math.pi
	
    #Setting the actual twist to send to the robot0
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = cmp(number,0) * TOP_SPEED

    #Set up a zero twist to go between the actual twists
    zero = Twist()
    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

    #Motion updates are going to be based on odometry instead
    #Then using our observations, we detect when we're in the correct state and stop
        #But it's not exactly stop
        #It's more like, as you get closer, slow down
	
	#The initial turn is done without any linear movement
    pubTwist.publish(zero)
    while abs(difference) > ANGULAR_LIMIT:
        #Once it reaches below a certain angle it slows at a ratio of the remaining angle over the limit
        if abs(difference) < ANGULAR_SLOW_LIMIT
            twist.angular.z = cmp(number,0) * TOP_SPEED * (difference / ANGULAR_SLOW_LIMIT)
	        
        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
        pubTwist.publish(twist)
        time.sleep(0.1)
        
        #Checks the new difference between current and desired angles
        currState = state
        difference = node.heading - currState.theta
        while difference > math.pi
            difference -= 2*math.pi
        while difference < -math.pi
            difference += 2*math.pi
    	
    pubTwist.publish(zero)
	
    #HANDLE FORWARDS MOTION
    while distance > LIMIT
        #Get the new position
        currState = state
	
        #Finding the distance to the next point
        distance = math.sqrt((currState.x - node.x) * (currState.x - node.x) + (currState.y - node.y) * (currState.y - node.y))
	
        #Finding the turn needed
        angle = atan2((currState.y-node.y),(currState.x-node.x))
        difference = angle - currState.theta
        difference = changeAngle(difference, node.forward)
		
        #Slow if within a certain limit of goal
        if distance < SLOW_LIMIT
            speed = TOP_SPEED * (difference / SLOW_LIMIT)
        else
            speed = TOP_SPEED
		
        #To make the robot go backwards
        if !node.forward
            speed = -speed
		
        #Slow if within a certain limit
        if difference < ANGULAR_SLOW_LIMIT
            angularSpeed = TOP_SPEED * (difference / ANGULAR_SLOW_LIMIT)
        else
            angularSpeed = TOP_SPEED

        twist.angular.z = angularSpeed
        twist.linear.x = speed

        #Publish the twist and wait a little to recalculate (not sure how long this should be for)		
        pubTwist.publish(twist)
        time.sleep(0.1)

    pubTwist.publish(zero)
	
def changeAngle(angle, forward)
	
    #Check difference is too large and readjusts value(i.e. goes over the bound between pi and -pi)
    while angle > math.pi
        angle = angle - 2*math.pi
    while angle < -math.pi
        angle = angle + 2*math.pi
	
    #Determine whether robot needs to go forwards or backwards and then depending on that which way it needs to turn
    if !forward
        if angle < 0
            angle = math.pi + angle
        else
            angle = math.pi - angle
    return angle
	
if __name__ == '__main__':
    control()
