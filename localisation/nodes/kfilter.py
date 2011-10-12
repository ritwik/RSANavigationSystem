#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons
from std_msgs.msg import Header

#Actual values
BX = [-1.0, -1.0, 0.0]
BY = [2.0, -2.0, 3.0]

pub = rospy.Publisher('State', State)

#x, y, theta
mean = array([[0.0], [0.0], [0.0]])
covar = array([[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, pi]])

def publishState():
    global pub
    global mean
    global covar
    #Chuck mean and covar into state message and publish
    pub.publish(State(Header(), mean[0, 0], mean[1, 0], mean[2, 0], covar[0, 0], covar[1, 1], covar[2, 2]))

def actionUpdate(move):
    #Set up necessary matrices
    F = array([[1, 0, -move.fwd * sin(theta)],
               [0, 1, move.fwd * cos(theta)],
               [0, 0, 1]])

    B = array([[cos(theta), 0],
               [sin(theta), 0],
               [0, 1]])

    R = array([[cos(theta), -sin(theta), 0]
               [sin(theta), cos(theta), 0],
               [0, 0, 1]])
    E = array([[0.1 + 0.01 * abs(move.fwd), 0, 0],
               [0, 0.1 + 0.01 * abs(move.fwd), 0],
               [0, 0, 5 * 180 / pi + 0.1 * abs(move.phi)]]) #Should these be squared?!
    Q = dot(dot(R, E), transpose(R))

    #Chuck action into matrix too
    u = array([[move.fwd, move.phi]])

    #Plug into formulae to get new mean and covariance
    mean = dot(dot(F, mean) + B, u)
    covar = dot(dot(F, covar), transpose(F)) + Q

    #Chuck mean and covar into state message and publish
    publishState()
    
def observationUpdate(data):
    global mean
    global covar

    for b in data.beacon:
        x = mean[0, 0]
        y = mean[1, 0]

        dx = BX[b.ID] - x
        dy = BY[b.ID] - y

        print dx
        print dy

        #Set up necessary matrices
        #Something tells me this is very wrong atm
        H = array([[0, 0, 0],
                   [dx / sqrt(dx ** 2 + dy ** 2), dy / sqrt(dx ** 2 + dy ** 2), 0],
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
        y = z - dot(H, mean)

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

        publishState()
        
def kfilter():
    global pub
    pub = rospy.Publisher('State', State)
    rospy.init_node('localisation', anonymous=True)
    rospy.Subscriber("BeaconScan", Beacons, observationUpdate)
    #rospy.Subscriber("Movement", Movements, actionUpdate)
    rospy.spin()

if __name__ == '__main__':
    kfilter()
