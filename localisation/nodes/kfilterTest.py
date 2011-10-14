#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons, Beacon
from control.msg import Move
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

def actionUpdate(move):
    global mean
    global covar

    #Set up necessary matrices
    F = array([[1, 0, -move.fwd * sin(mean[2, 0])],
               [0, 1, move.fwd * cos(mean[2, 0])],
               [0, 0, 1]])

    print "F = " + str(F)

    #B = array([[cos(theta), 0],
    #           [sin(theta), 0],
    #           [0, 1]])

    R = array([[cos(mean[2, 0]), -sin(mean[2, 0]), 0],
               [sin(mean[2, 0]), cos(mean[2, 0]), 0],
               [0, 0, 1]])

    print "R = " + str(R)

    E = array([[(0.1 + 0.01 * abs(move.fwd)) ** 2, 0, 0],
               [0, (0.1 + 0.01 * abs(move.fwd)) ** 2, 0],
               [0, 0, (5 * 180 / pi + 0.1 * abs(move.phi)) ** 2]])

    print "E = " + str(E)

    Q = dot(dot(R, E), transpose(R))

    print "Q = " + str(Q)

    #Chuck action into matrix too
    u = array([[move.fwd, move.phi]])

    print "u = " + str(u)

    #Plug into formulae to get new mean and covariance
    mean = array([[mean[0,0] + move.fwd * cos(mean[2, 0])], [mean[1,0] + move.fwd * sin(mean[2, 0])], [mean[2,0] + move.phi]])

    print "mean = " + str(mean)

    covar = dot(dot(F, covar), transpose(F)) + Q

    print "covar = " + str(covar)

def observationUpdate(data):
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
b0 = Beacon(0, 0.0, 0.0, 5 ** 0.5, atan2(-2, 1))
b2 = Beacon(1, 0.0, 0.0, 5 ** 0.5, atan2(2, 1))
msg = Beacons(Header(), 2, [b0, b2])

#At (0,0) facing pi/2 (90 degrees to the left)
#b0 = Beacon(0, 0.0, 0.0, 5 ** 0.5, atan2(1, 2))
#b1 = Beacon(1, 0.0, 0.0, 5 ** 0.5, atan2(1, -2))
#b2 = Beacon(2, 0.0, 0.0, 3, atan2(-3, 0))
#msg = Beacons(Header(), 3, [b0, b1, b2])

for i in range(0,50):
    observationUpdate(msg)

#Move 1 metre forwards from where it was positioned above
#msg = Move(Header(), 1.0, 0)
#actionUpdate(msg)

#Move 2 metres backwards from where it was positioned above
msg = Move(Header(), -2.0, 0)
actionUpdate(msg)

#Originally at (0,0) facing pi/2 (90 degrees to the left) then do 2 metres backwards
#b0 = Beacon(0, 0.0, 0.0, 17 ** 0.5, atan2(1, 4))
#b1 = Beacon(1, 0.0, 0.0, 1, atan2(1, 0))
#b2 = Beacon(2, 0.0, 0.0, 13 ** 0.5, atan2(-3, 2))
#msg = Beacons(Header(), 3, [b0, b1, b2])

#for i in range(0,50):
#    observationUpdate(msg)
