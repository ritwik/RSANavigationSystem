#!/usr/bin/env python
import roslib; roslib.load_manifest('control')
import rospy
import time
import math
from math import atan2

import pathplanner

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from localisation.msg import State
from beaconfinder.msg import Beacons
from control.msg import Pause

import sys
import time

#Need a limit so the robot stops trying to get closer (0.1m and 5 degrees) feel free to change
LIMIT = 0.2
#ANGULAR_LIMIT = ((5 / 180) * math.pi) 
ANGULAR_LIMIT = ((5.0 / 180) * math.pi) 

TIME_LIMIT = 10

#This is the top speed of the robot
TOP_SPEED = 0.4
TOP_ANGULAR_SPEED = 1.0

#This is the distance where the robot begins to slow (0.5m and 45 degrees) they may be wrong, feel free to change them
SLOW_LIMIT = 0.5
ANGULAR_SLOW_LIMIT = math.pi / 4

SLOW_CONSTANT = 0.1

VAR_THRESH = 5000000.5 # maybe this is too high

state = None
settings = None

pubTwist = None
pubPause = None
twoBeacons = False

def callbackState(data):
    global state
    state = data

#def run():
#    print "Waiting for startup keypress"
#    line = sys.stdin.readline()
#
#    #Start by spinning around to determine where you are
#    spinAround()
#
#    plan = pathplanner.planPath(state)
#
#    plan.path.pop(0)
#
#    for node in plan.path:
#        print (node)
#
#    for node in plan.path:
#        print "Waiting for next point keypress:", node
#        # here we should connect
#        sys.stdin.readline()
#        drive(node)
#        # plan = pathplanner.planPath(state)
#

def beaconsCallback(beacons):
    global twoBeacons

    if beacons.numBeacons >= 2:
        twoBeacons = True
    else:
        twoBeacons = False

def spinAround():
    global twoBeacons

    #Setting the actual twist to send to the robot
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = TOP_ANGULAR_SPEED / 2

    twist2 = Twist()
    twist2.linear.x = TOP_SPEED; twist2.linear.y = 0; twist2.linear.z = 0
    twist2.angular.x = 0; twist2.angular.y = 0; twist2.angular.z = 0

    #Set up a zero twist to go between the actual twists
    zero = Twist()
    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

	#The initial turn is done without any linear movement
    pubTwist.publish(zero)
#    seenTwoBeacons = False
    #while not twoBeacons and not seenTwoBeacons:
        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
    for i in range(0,65):
        pubTwist.publish(twist)
        time.sleep(0.2)

        if (twoBeacons):
            time.sleep(2)
            break
#
#        if not seenTwoBeacons:
#            pubTwist.publish(twist2)

    pubTwist.publish(zero)
    
#def lostSpinAround(node):
#    #Setting the actual twist to send to the robot
#    twist = Twist()
#    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
#    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1
#
#    #Set up a zero twist to go between the actual twists
#    zero = Twist()
#    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
#    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0
#    
#	#The initial turn is done without any linear movement
#    pubTwist.publish(zero)
#    while (state.xCovar > (VAR_THRESH*.75) or state.yCovar > VAR_THRESH*.75 or (state.thetaCovar > 10.0/180*math.pi*.75)):
#        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
#        pubTwist.publish(twist)
#    
#    drive(node)
#    pubTwist.publish(zero)
#    print "we've finished spinning to localize"
#

def greedySearch(points):
    global state

    currState = state
    minDistance = 0
    nextPoint = None

    for point in points:
        distance = math.sqrt((currState.x - point[0]) ** 2 + (currState.y - point[1]) ** 2)

        if nextPoint == None or distance < minDistance:
            minDistance = distance
            nextPoint = point

    return nextPoint

def control():
    global pubTwist
    global state
    global pubPause

    rospy.init_node('control')
    pubTwist = rospy.Publisher('cmd_vel', Twist)
    pubPause = rospy.Publisher('Pause', Pause)
    rospy.Subscriber("State", State, callbackState)
    rospy.Subscriber("BeaconScan", Beacons, beaconsCallback)

    f = open('/home/akeswani/ros_workspace/RSANavigationSystem/control/nodes/points', 'r') #In our starting script we will want to copy given points file to this points file
    points = []
    for i in range(0, 5):
        line = f.readline()
        splitLine = line.split(' ')
        points.append([float(splitLine[0]) / 100.0, float(splitLine[1]) / 100.0])
    print points

    while points != []:
        print "Waiting for keypress"
        pubPause.publish(Pause(True))
        sys.stdin.readline()
        pubPause.publish(Pause(False))

        spinAround()

        currState = state
        nextPoint = greedySearch(points)
        points.remove(nextPoint)
        drive(nextPoint)

        print "Now at point", nextPoint

def drive(point):
    global state
    global pubPause

    print "Going to point", point

    currState = state

    #Finding the distance to the next point
    distance = math.sqrt((currState.x - point[0]) ** 2 + (currState.y - point[1]) ** 2)
	 
    if distance < LIMIT:
        return
    
    #ROTATION
    angleToTurn = getAngleToTurn(point, currState)

    #Setting the actual twist to send to the robot0
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = cmp(angleToTurn,0) * TOP_ANGULAR_SPEED 

    #Set up a zero twist to go between the actual twists
    zero = Twist()
    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0

	#The initial turn is done without any linear movement
    pubTwist.publish(zero)

    while abs(angleToTurn) > ANGULAR_LIMIT:
        #Once it reaches below a certain angle it slows at a ratio of the remaining angle over the limit
        if abs(angleToTurn) < ANGULAR_SLOW_LIMIT:
            twist.angular.z = TOP_ANGULAR_SPEED * (angleToTurn / ANGULAR_SLOW_LIMIT) 
	    
        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
        pubTwist.publish(twist)
        time.sleep(0.1)
        
        #Checks the new difference between current and desired angles
        currState = state

        angleToTurn = getAngleToTurn(point, currState)
    
    pubTwist.publish(zero)

    #FORWARDS
    while distance > LIMIT:
        currState = state
        
        #Finding the distance to the next point
        distance = math.sqrt((currState.x - point[0]) ** 2 + (currState.y - point[1]) ** 2)

        #Finding the turn needed
        angleToTurn = getAngleToTurn(point, currState)

        #Slow if within a certain limit of goal
        if abs(distance) < SLOW_LIMIT:
            forwardSpeed = TOP_SPEED * (distance / SLOW_LIMIT) + SLOW_CONSTANT
            angularSpeed = 0
            if abs(angleToTurn) >= math.pi / 2:
                print "Overshot on angle"
                break
        else:
            forwardSpeed = TOP_SPEED
            if abs(angleToTurn) < ANGULAR_SLOW_LIMIT:
                angularSpeed = TOP_ANGULAR_SPEED * (angleToTurn / ANGULAR_SLOW_LIMIT)
            else:
                angularSpeed = TOP_ANGULAR_SPEED * cmp(angleToTurn, 0)
		
        #Slow if within a certain limit
        twist.angular.z = angularSpeed
        twist.linear.x = forwardSpeed

        #Publish the twist and wait a little to recalculate (not sure how long this should be for)		
        pubTwist.publish(twist)
        time.sleep(0.1)

    pubTwist.publish(zero)

def getAngleToTurn(point, currState):
    heading = atan2((point[1] - currState.y),(point[0] - currState.x))
    angleToTurn = heading - currState.theta
    while angleToTurn > math.pi:
        angleToTurn = angleToTurn - 2*math.pi
    while angleToTurn < -math.pi:
        angleToTurn = angleToTurn + 2*math.pi

    return angleToTurn

#def drive(node):
#    global state
#
#    currState = state
#
#    #Finding the distance to the next point
#    distance = math.sqrt((currState.x - node.x) ** 2 + (currState.y - node.y) ** 2)
#	 
#    if distance < LIMIT:
#        return
#    
#    #HANDLE ROTATION
#        #Recalculate heading to make sure it is correct if the robot is in au unexpected place
#    #SOMETHING IS WRONG!
#    newNodeHeading = atan2(node.y - currState.y, node.x - currState.x)
#    node.heading = changeAngle(newNodeHeading, node.forward)
#    print node.heading
#
#    #	sys.stdin.readline()
#
#    #Finding the turn needed
#    difference = node.heading - currState.theta
#    while difference > math.pi:
#        difference -= 2 * math.pi
#    while difference < -math.pi:
#        difference += 2 * math.pi
#
#    #Setting the actual twist to send to the robot0
#    twist = Twist()
#    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
#    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = cmp(difference,0) * TOP_ANGULAR_SPEED 
#
#    #Set up a zero twist to go between the actual twists
#    zero = Twist()
#    zero.linear.x = 0; zero.linear.y = 0; zero.linear.z = 0
#    zero.angular.x = 0; zero.angular.y = 0; zero.angular.z = 0
#
#	#The initial turn is done without any linear movement
#    pubTwist.publish(zero)
#    print "Let's spin around first... to get the angle right"
#    while abs(difference) > ANGULAR_LIMIT:
#        print "difference is " + str(difference) + " node.heading is " + str(node.heading) + " currState.theta " + str(currState.theta), str(ANGULAR_LIMIT), str(ANGULAR_SLOW_LIMIT) 
#        #Once it reaches below a certain angle it slows at a ratio of the remaining angle over the limit
#        if abs(difference) < ANGULAR_SLOW_LIMIT:
#            twist.angular.z = TOP_ANGULAR_SPEED * (difference / ANGULAR_SLOW_LIMIT) 
#	    
#        print "angular speed is " + str(twist.angular.z)
#
#        #Publish the twist and wait a little to recalculate (not sure how long this should be for)
#        pubTwist.publish(twist)
#        time.sleep(0.1)
#        
#        #Checks the new difference between current and desired angles
#        currState = state
#
#        difference = node.heading - currState.theta # you can calculate this 
#
#        while difference > math.pi:
#            difference = difference - 2 * math.pi
#        while difference < -math.pi:
#            difference = difference + 2 * math.pi
#    	
#    pubTwist.publish(zero)
#
#    #Get the starting heading of the robot, 
#    startHeading = currState.theta
#    
#    print "Let's actually move there"
#    print node
#
#    #HANDLE FORWARDS MOTION
#    start = time.time()
#    while distance > LIMIT:
#        end = time.time()
#        
#        #This enforces a time limit
#        #if end - start > TIME_LIMIT:
#        #    break
#            
#        #Get the new position
#        currState = state
#        
#        #if (currState.xCovar > VAR_THRESH or currState.yCovar > VAR_THRESH):
#        #  print "We're a little lost about position"
#        #  lostSpinAround(node)
#        #  return
#        #else:
#        #  if (currState.thetaCovar > 10.0/180*math.pi):
#        #    print "We're a little lost about the angle"
#        #    lostSpinAround(node)
#        #    return        
#	
#        #Finding the distance to the next point
#        distance = math.sqrt((currState.x - node.x) * (currState.x - node.x) + (currState.y - node.y) * (currState.y - node.y))
#        print distance
#
#        #Finding the turn needed
#        angle = atan2((node.y - currState.y),(node.x - currState.x)) #atan2((currState.y-node.y),(currState.x-node.x))
#        angle = changeAngle(angle, node.forward)
#        difference = angle - currState.theta
#        while difference > math.pi:
#            difference = difference - 2*math.pi
#        while difference < -math.pi:
#            difference = difference + 2*math.pi
#        print "Difference:", difference
#
#		#This will mean that it has overshot and is turning around, which is stupid
#        overshootDifference = abs(startHeading - angle)
#        while overshootDifference > math.pi:
#            overshootDifference = overshootDifference - 2 * math.pi
#        print "Overshoot difference: ", overshootDifference
#        if abs(overshootDifference) > math.pi / 2:
#		    break
#        
#        #Slow if within a certain limit of goal
#        if abs(distance) < SLOW_LIMIT:
#            speed = TOP_SPEED * (distance / SLOW_LIMIT) + SLOW_CONSTANT
#        else:
#            speed = TOP_SPEED
#		
#        #To make the robot go backwards
#        if not node.forward:
#            speed = -speed
#		
#        #Slow if within a certain limit
#        if abs(difference) < ANGULAR_SLOW_LIMIT:
#            angularSpeed = TOP_ANGULAR_SPEED * (difference / ANGULAR_SLOW_LIMIT)
#        else:
#            angularSpeed = TOP_ANGULAR_SPEED * cmp(difference, 0)
#
#        twist.angular.z = angularSpeed
#        twist.linear.x = speed
#
#        #Publish the twist and wait a little to recalculate (not sure how long this should be for)		
#        pubTwist.publish(twist)
#        time.sleep(0.1)
#
#    pubTwist.publish(zero)
#	
#
#def changeAngle(angle, forward):
#    #Determine whether robot needs to go forwards or backwards and then depending on that which way it needs to turn
#    print forward
#    print angle
#    if not forward:
#        if angle < 0:
#            angle = math.pi + angle
#        else:
#            angle = angle - math.pi
#
#    print forward
#    print angle
#
#    #Check difference is too large and readjusts value(i.e. goes over the bound between pi and -pi)
#    while angle > math.pi: 
#        angle = angle - 2 * math.pi
#    while angle < -math.pi: 
#        angle = angle + 2 * math.pi
#	
#    return angle
	
if __name__ == '__main__':
    control()
