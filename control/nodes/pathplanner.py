#!/usr/bin/python
import math
from math import atan2,pi

#Algo
#Construct a tree
#Breadth first search
#Have a FIFO queue of nodes to search
#Once you reach the leaves (no more to search) store the value
#Repeat for all
#Find minimum

FORWARD_SPEED = 2.17
TURN_SPEED = 0.64

pQueue = []
fullPaths = []

class Node:
    def __init__(self, x, y, heading, remaining, path, time, forward):
        self.x = x
        self.y = y
        self.heading = heading
        self.remaining = remaining
        self.path = path
        self.time = time
        self.forward = forward

    def __repr__(self):
        return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.heading) + ", " + str(self.forward) + "]"
    

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __repr__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

def newTimeAndHeading(fromNode, toCoordinate):
    dy = fromNode.y - toCoordinate.y
    dx = fromNode.x - toCoordinate.x

    distance = math.sqrt(dx ** 2 + dy ** 2)

    newHeading = atan2(dy, dx)   
    currHeading = fromNode.heading
    if newHeading < 0:
        newHeading = newHeading + 2 * pi
    if currHeading < 0:
        currHeading = currHeading + 2 * pi

    angleBetween = newHeading - currHeading

    if angleBetween > 0:
        if angleBetween <= pi:
            fwdAngle = angleBetween
            bwdAngle = pi - angleBetween
        else:
            fwdAngle = angleBetween - 2 * pi
            bwdAngle = angleBetween - pi
    else:
        if angleBetween >= -pi:
            fwdAngle = angleBetween
            bwdAngle = pi - angleBetween
        else:
            fwdAngle = angleBetween + 2 * pi
            bwdAngle = angleBetween + pi
    
    #4 options: turn positive or negative, move backwards or forwards
    #Positive forwards, positive backwards, negative forwards, negative backwards
    newHeading = newHeading - pi
    if newHeading > 0:
        bwdHeading = newHeading - pi
    else :
        bwdHeading = newHeading + pi

    fwdTime = distance / FORWARD_SPEED + fwdAngle / TURN_SPEED
    bwdTime = distance / FORWARD_SPEED + bwdAngle / TURN_SPEED

    while newHeading >= pi:
        newHeading = newHeading - 2 * pi
    while newHeading < -pi:
        newHeading = newHeading + 2 * pi

    while bwdHeading >= pi:
        bwdHeading = bwdHeading - 2 * pi
    while bwdHeading < -pi:
        bwdHeading = bwdHeading + 2 * pi

    headingToCentreAtStart = atan2((0 - fromNode.y), (0 - fromNode.x))
    print "Going to ", toCoordinate.x, toCoordinate.y, "from ", fromNode.x, fromNode.y
    print headingToCentreAtStart
    print bwdHeading - headingToCentreAtStart
    print newHeading - headingToCentreAtStart

    if abs(bwdHeading - headingToCentreAtStart) < abs(newHeading - headingToCentreAtStart):
        return bwdTime, bwdHeading, False
    else:
        return fwdTime, newHeading, True
        
def findPath(startNode):
    pQueue.append(startNode)

    while len(pQueue) != 0:
        currNode = pQueue.pop(0)

        if not currNode.remaining:
            fullPaths.append(currNode)
        else:
            for unexplored in currNode.remaining:
                time, heading, forward = newTimeAndHeading(currNode, unexplored)
                #print time, heading, forward

                newPath = list(currNode.path)
                newPath.append(currNode)
                pQueue.append(Node(unexplored.x, unexplored.y, heading, [n for n in currNode.remaining if n != unexplored], newPath, time, forward))

                #newPath = list(currNode.path)
                #newPath.append(currNode)
                #pQueue.append(Node(unexplored.x, unexplored.y, bwdHeading, [n for n in currNode.remaining if n != unexplored], newPath, bwdTime, False))
            
def chooseBestPath():
    minTime = 0
    bestPath = None
    for p in fullPaths:
        if p.time < minTime or bestPath==None:
            minTime = p.time
            bestPath = p
    
    bestPath.path.append(bestPath)
    print "The best path is: ", bestPath.path

    return bestPath

def planPath(state):
    points = []
    f = open('/home/akeswani/ros_workspace/RSANavigationSystem/control/nodes/points', 'r') #In our starting script we will want to copy given points file to this points file
    for i in range(0, 5):
        line = f.readline()
        splitLine = line.split(' ')
        points.append(Coordinate(float(splitLine[0]) / 100.0, float(splitLine[1]) / 100.0))
    print points

    print "Finding all paths"
    findPath(Node(state.x, state.y, state.theta, points, [], 0.0, True))

    print "Choosing best path"
    bestPath = chooseBestPath()

    return bestPath
