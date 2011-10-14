#!/usr/bin/python
import roslib; roslib.load_manifest('pathplanner')
import rospy

import math
from math import atan2,pi

from pathplanner.msg import Path, PathNode

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

def newTimesAndHeadings(fromNode, toCoordinate):
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

    return fwdTime, newHeading, bwdTime, bwdHeading

def findPath(startNode):
    pQueue.append(startNode)

    while len(pQueue) != 0:
        currNode = pQueue.pop(0)

        if not currNode.remaining:
            fullPaths.append(currNode)
        else:
            for unexplored in currNode.remaining:
                fwdTime, fwdHeading, bwdTime, bwdHeading = newTimesAndHeadings(currNode, unexplored)
                print fwdTime, fwdHeading, bwdTime, bwdHeading

                newPath = list(currNode.path)
                newPath.append(currNode)
                pQueue.append(Node(unexplored.x, unexplored.y, fwdHeading, [n for n in currNode.remaining if n != unexplored], newPath, fwdTime))

                newPath = list(currNode.path)
                newPath.append(currNode)
                pQueue.append(Node(unexplored.x, unexplored.y, bwdHeading, [n for n in currNode.remaining if n != unexplored], newPath, bwdTime))
            
def chooseBestPath():
    minTime = 0
    bestPath = None
    for p in fullPaths:
        if p.time < minTime or bestPath==None:
            minTime = p.time
            bestPath = p
    
    pathNodes = []
    for n in bestPath.path:
        pathNodes.append(PathNode(n.x, n.y, n.heading))
    pathNodes.append(PathNode(bestPath.x, bestPath.y, bestPath.heading))

    print "The best path is: ", pathNodes
    #pub.publish(new Path(Header(), pathNodes))

if __name__ == "__main__":
    #Open file, get nodes
    c1 = Coordinate(1,2)
    c2 = Coordinate(2,1)
    c3 = Coordinate(3,3)

    print "Finding all paths"
    findPath(Node(0, 0, 0, [c1,c2,c3], [], 0.0))

    print "Choosing best path"
    chooseBestPath()
