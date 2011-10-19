#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons, Beacon
from std_msgs.msg import Header

#Actual values
BX = [-1.0, -1.0, 3.0]
BY = [2.0, -2.0, 0.0]

pub = rospy.Publisher('State', State)

#x, y, theta
mean = array([[1.1], [-2.1], [0.501]])
covar = array([[25000.0, 0.0, 0.0], [0.0, 25000.0, 0.0], [0.0, 0.0, 25000.0]])

def publishState():
    global pub
    global mean
    global covar
    #Chuck mean and covar into state message and publish
    pub.publish(State(Header(), mean[0, 0], mean[1, 0], mean[2, 0], covar[0, 0], covar[1, 1], covar[2, 2]))

def actionUpdate(dx, dy, dt):
    global mean
    global covar

    #Set up necessary matrices
    F = array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])

    print "F = " + str(F)

    Q = array([[(0.1 + 0.01 * abs(dx)) ** 2, 0, 0],
               [0, (0.1 + 0.01 * abs(dy)) ** 2, 0],
               [0, 0, (5 * 180 / pi + 0.05 * abs(dt)) ** 2]])

    print "Q = " + str(Q)

    #Plug into formulae to get new mean and covariance
    mean = array([[mean[0,0] + dx], [mean[1,0] + dy], [mean[2,0] + dt]])

    print "mean = " + str(mean)

    covar = dot(dot(F, covar), transpose(F)) + Q

    print "covar = " + str(covar)

    #Chuck mean and covar into state message and publish
    publishState()

def observationUpdate(data):
    global mean
    global covar

    for b in data.beacon:
        x = mean[0, 0]
        y = mean[1, 0]

        dx = b.x - x
        dy = b.y - y

        print "Dx = " + str(dx)
        print "Dy = " + str(dy)

        #Set up necessary matrices
        H = array([[0, 0, 0],
                   [-dx / sqrt(dx ** 2 + dy ** 2), -dy / sqrt(dx ** 2 + dy ** 2), 0],
                   [dy / (dx ** 2 + dy ** 2), -dx / (dx ** 2 + dy ** 2), -1]])

        print "H = " + str(H)

        Hx = array([[0], [sqrt(dx ** 2 + dy ** 2)], [atan2(dy, dx) - mean[2, 0]]])

        print "Hx = " + str(Hx)

        R = array([[1, 1, 1],
                   [0, 0.02 + 0.01 * (dx ** 2 + dy ** 2), 0],
                   [0, 0, 0.1]]) 

        print "R = " + str(R)

        #Chuck beacon observation into matrix
        z = array([[0], [b.distance], [b.angle]])

        print "z = " + str(z)
        print b.angle * 180 / pi

        #Plug into formulae to get new mean and covariance
        y = z - Hx #dot(H, mean)
        while y[2,0] >= pi:
            y[2,0] = y[2,0] - 2 * pi
        while y[2,0] < -pi:
            y[2,0] = y[2,0] + 2 * pi

        print "y = " + str(y)

        S = dot(dot(H, covar), transpose(H)) + R
    
        print "S = " + str(S)

        K = dot(dot(covar, transpose(H)), linalg.inv(S))

        print "K = " + str(K)

        mean = mean + dot(K, y)

        print "mean = " + str(mean)
        while mean[2,0] >= pi:
            mean[2,0] = mean[2,0] - 2 * pi
        while mean[2,0] < -pi:
            mean[2,0] = mean[2,0] + 2 * pi

        covar = dot((eye(3) - dot(K, H)), covar)

        print "covar = " + str(covar)

def oldObservationUpdate(data):
    global mean
    global covar

    for b in data.beacon:
        print b.ID, b.distance, b.angle

        x = mean[0, 0]
        yy = mean[1, 0]

        dx = BX[b.ID] - x
        dy = BY[b.ID] - yy

        #Set up necessary matrices
        H = array([[0, 0, 0],
                   [-dx / sqrt(dx ** 2 + dy ** 2), -dy / sqrt(dx ** 2 + dy ** 2), 0],
                   [dy / (dx ** 2 + dy ** 2), -dx / (dx ** 2 + dy ** 2), -1]])

        print "H = " + str(H)

        Hx = array([[0], [sqrt(dx ** 2 + dy ** 2)], [atan2(dy, dx) - mean[2, 0]]])

        print "Hx = " + str(Hx)

        R = array([[1, 1, 1],
                   [0, 0.02 + 0.01 * (dx ** 2 + dy ** 2), 0],
                   [0, 0, 0.1]]) 

        print "R = " + str(R)

        #Chuck beacon observation into matrix
        z = array([[0], [b.distance], [b.angle]])

        print "z = " + str(z)
        print b.angle * 180 / pi

        #Plug into formulae to get new mean and covariance

        y = z - Hx #dot(H, mean)
        while y[2,0] >= pi:
            y[2,0] = y[2,0] - 2 * pi
        while y[2,0] < -pi:
            y[2,0] = y[2,0] + 2 * pi

        print "y = " + str(y)

        S = dot(dot(H, covar), transpose(H)) + R
    
        print "S = " + str(S)

        K = dot(dot(covar, transpose(H)), linalg.inv(S))

        print "K = " + str(K)

        mean = mean + dot(K, y)
        while mean[2,0] >= pi:
            mean[2,0] = mean[2,0] - 2 * pi
        while mean[2,0] < -pi:
            mean[2,0] = mean[2,0] + 2 * pi

        print "mean = " + str(mean)

        covar = dot((eye(3) - dot(K, H)), covar)

        print "covar = " + str(covar)

#At (0,0) facing pi (backwards)
b0 = Beacon(0, -1.0, 2.0, 5 ** 0.5, atan2(-2, 1))
b2 = Beacon(1, -1.0, -2.0, 5 ** 0.5, atan2(2, 1))
msg = Beacons(Header(), 2, [b0, b2])

#At (0,0) facing pi/2 (90 degrees to the left)
#b0 = Beacon(0, 0.0, 0.0, 5 ** 0.5, atan2(1, 2))
#b1 = Beacon(1, 0.0, 0.0, 5 ** 0.5, atan2(1, -2))
#b2 = Beacon(2, 0.0, 0.0, 3, atan2(-3, 0))
#msg = Beacons(Header(), 3, [b0, b1, b2])

for i in range(0,50):
    observationUpdate(msg)    

#
#actionUpdate(1, 1, 0)

