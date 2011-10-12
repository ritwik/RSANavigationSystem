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
    def __init__(self, x, y, heading, remaining, path, time):
        self.x = x
        self.y = y
        self.heading = heading
        self.remaining = remaining
        self.path = path
        self.time = time

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class TimesAndHeadings:
    def __init__(self, fwdTime, fwdHeading, bwdTime, bwdHeading):
        self.fwdTime = fwdTime
        self.fwdHeading = fwdHeading
        self.bwdTime = bwdTime
        self.bwdHeading = bwdHeading

def newTimesAndHeadings(fromNode, toCoordinate):
    dy = fromNode.y - toCoordinate.y
    dx = fromNode.x - toCoordinate.x

    distance = math.sqrt(dx** 2 + dy ** 2)

    newHeading = atan2(dy, dx)   
    if newHeading < 0:
        newHeading = newHeading + 2 * math.pi
    if fromNode.heading < 0:
        fromNode.heading = fromNode.heading + 2 * math.pi

    angleBetween = newHeading - fromNode.heading

    if angleBetween > 0:
        if angleBetween <= pi:
            fwdAngle = angleBetween
            bwdAngle = pi - angleBetween
        else:
            fwdAngle = angleBetween - 2*pi
            bwdAngle = angleBetween - pi
    else:
        if angleBetween >= -pi:
            fwdAngle = angleBetween
            bwdAngle = pi - angleBetween
        else:
            fwdAngle = angleBetween + 2*pi
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

    return TimesAndHeadings(fwdTime, newHeading, bwdTime, bwdHeading)

def findPath(startNode):
    pQueue.append(startNode)

    while len(pQueue) != 0:
        currNode = pQueue.pop(0)
        if not currNode.remaining:
            fullPaths.append(currNode)
        else:
            for unexplored in currNode.remaining:
                timesAndHeadings = newTimesAndHeadings(currNode, unexplored)
                
                #pQueue.append(Node(unexplored.x, unexplored.y, timesAndHeadings.fwdHeading, [n for n in currNode.remaining if n != unexplored], currNode.path.append(currNode), timesAndHeadings.fwdTime)
                #pQueue.append(Node(unexplored.x, unexplored.y, timesAndHeadings.bwdHeading, [n for n in currNode.remaining if n != unexplored], currNode.path.append(currNode), timesAndHeadings.bwdTime)
            


def chooseBestPath():
    minTime = 0
    bestPath = None
    for p in fullPaths:
        if p.time < minTime or bestPath==None:
            minTime = p.time
            bestPath = p
    
    pathNodes = []
    for n in bestPath.path:
        pathNodes.append(Node(n.x, x.y, n.heading))
        #print "x="n.x, ", ", "y=",n.y," ","heading=",n.heading
    pathNodes.append(bestPath.x, bestPath.y, bestPath.heading)
    print "The best path is: ",pathNodes
    #pub.publish(new Path(Header(), pathNodes))

if __name__ == "__main__":
    #Open file, get nodes
    print "In main"
    c1 = Coordinate(1,2)
    c2 = Coordinate(2,1)
    c3 = Coordinate(3,3)
    print "Finding all paths"
    print fullPaths
    findPath(Node(0, 0, 0, [c1,c2,c3],[],0.0))
    print "Choosing best path"
    chooseBestPath()
