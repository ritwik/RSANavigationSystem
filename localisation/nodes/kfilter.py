#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons

BX = [-100, 0, -100]
BY = [200, 300, -200]

pub = rospy.Publisher('State', State)

#x, y, theta
mean = array([[0.0, 0.0, 0.0]])
covar = array([[250000.0, 250000.0, 250000.0]])

def publishState():
    global pub
    #Chuck mean and covar into state message and publish
    pub.publish(State(Header(), mean[0, 0], mean[0, 1], mean[0, 2], covar[0, 0], covar[0, 1], covar[0, 2]))

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
    Q = R * E * transpose(R)

    #Chuck action into matrix too
    u = array([[move.fwd, move.phi]])

    #Plug into formulae to get new mean and covariance
    mean = F * mean + B * u
    covar = F * covar * transpose(F) + Q

    #Chuck mean and covar into state message and publish
    publishState()
    
def observationUpdate(data):
    global mean
    global covar

    for b in data.beacon:
        x = mean[0, 0]
        y = mean[0, 1]

        dx = x - BX[b.ID]
        dy = y - BY[b.ID]

        print dx
        print dy

        #Set up necessary matrices
        H = array([[0, 0, 0],
                   [dx / ((x ** 2 - 2 * x * BX[b.ID]) ** 0.5), dy / ((y ** 2 - 2 * y * BY[b.ID]) ** 0.5), 0],
                   [-dy / (dx ** 2 + dy ** 2), dx / (dx ** 2 + dy ** 2), -1]])

        R = array([[0, 0, 0],
                   [0, 0.1 + 0.01 * b.distance, 0],
                   [0, 0, 5 * 180 / pi + 0.1 * abs(b.angle)]]) #Should these be squared?!

        print H
        #Chuck beacon observation into matrix
        z = array([[b.ID, b.distance, b.angle]])

        #Plug into formulae to get new mean and covariance
        y = z - H * mean
        S = H * covar * transpose(H) + R
        K = covar * transpose(H) * linalg.inv(S)

        mean = mean + K * y
        covar = (eye(3) - K * H) * covar

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
