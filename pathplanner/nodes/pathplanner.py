#!/usr/bin/python

import math

#Algo
#Construct a tree
#Breadth first search
#Have a FIFO queue of nodes to search
#Once you reach the leaves (no more to search) store the value
#Repeat for all
#Find minimum

FORWARD_SPEED = 
TURN_SPEED = 

pQueue = []
fullPaths = []

class Node:
    def __init__(self, x, y, heading, remaining, path, time):
        self.x = x
        self.y = y
        self.heading = heading

        self.remaining = remaining
        self.path = path

        self.time = self.time

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
    
    #4 options: turn positive or negative, move backwards or forwards
    #Positive forwards, positive backwards, negative forwards, negative backwards
    angleBetween = newHeading - fromNode.heading

    if (angleBetween > math.pi):
        fwdTurnAmount = 2 * math.pi - angleBetween
    else:
        fwdTurnAmount = angleBetween
    fwdHeading = newHeading

    if (angle ):
        bwdTurnAmount = 
    else:
        bwdTurnAmount = 
    bwdHeading = newHeading + math.pi?

    turnOptions = [
        (newHeading - fromNode.heading)),
        (2 * math.pi - (newHeading - fromNode.heading)),
    ]

        (pi - (newHeading - fromNode.heading))
        (pi + (newHeading - fromNode.heading)),
    ]

    fwdTime = distance / FORWARD_SPEED + fwdTurnAmount / TURN_SPEED
    bwdTime = distance / FORWARD_SPEED + bwdTurnAmount / TURN_SPEED

    return TimesAndHeadings(fwdTime, fwdHeading, bwdTime, bwdHeading)

def findPath(startNode):
    pQueue.append(startNode)

    while len(pQueue) != 0:
        currNode = pQueue.dequeue()
        if (currNode.remaining.isEmpty()):
            fullPaths.append(currNode)
        else:
            for unexplored in currNode.remaining:
                timesAndHeadings = newTimesAndHeadings(currNode, unexplored)
                
                pQueue.add(Node(unexplored.x, unexplored.y, timesAndHeadings.fwdHeading, 
                                [n for n in currNode.remaining if n != unexplored], 
                                currNode.path.add(currNode), timesAndHeadings.fwdTime)
                pQueue.add(Node(unexplored.x, unexplored.y, timesAndHeadings.bwdHeading, 
                                [n for n in currNode.remaining if n != unexplored], 
                                currNode.path.add(currNode), timesAndHeadings.bwdTime)
            
def chooseBestPath():
    #May need to create a Node message?
    #Yep
    pub.publish(new Path())

def __main__():
    #Open file, get nodes
    findPath(Node(x, y, heading, [#Add all points]))
    sendBestPath()
