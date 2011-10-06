#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons
from std_msgs.msg import Header

#Actual values
BX = [-1.0, 0.0, -1.0]
BY = [2.0, 3.0, -2.0]

pub = rospy.Publisher('State', State)

#x, y, theta
mean = array([[0.0], [0.0], [0.0]])
covar = array([[25000.0, 0.0, 0.0], [0.0, 25000.0, 0.0], [0.0, 0.0, 2 * pi]])

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

        dx = x - BX[b.ID]
        dy = y - BY[b.ID]

        #Set up necessary matrices
        H = array([[0, 0, 0],
                   [dx / ((dx ** 2 + dy ** 2) ** 0.5), dy / ((dx ** 2 + dy ** 2) ** 0.5), 0],
                   [-dy / (dx ** 2 + dy ** 2), dx / (dx ** 2 + dy ** 2), -1]])

        print "H = " + str(H)

        R = array([[1, 1, 1],
                   [0, 0.3 + 0.01 * b.distance, 0],
                   [0, 0, pi / 18 + 0.1 * abs(b.angle)]]) #Should these be squared?!

        print "R = " + str(R)

        #Chuck beacon observation into matrix
        z = array([[0], [b.distance], [b.angle]])

        print "z = " + str(z)

        #Plug into formulae to get new mean and covariance
        y = z - dot(H, mean)

        print "y = " + str(y)

        S = dot(dot(H, covar), transpose(H)) + R
    
        print "S = " + str(S)

        K = dot(dot(covar, transpose(H)), linalg.inv(S))

        print "K = " + str(K)

        mean = mean + dot(K, y)

        print "mean = " + str(mean)

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
