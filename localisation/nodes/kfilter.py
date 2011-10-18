#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion

pub = rospy.Publisher('State', State)

#x, y, theta
mean = array([[0.0], [0.0], [0.0]])
covar = array([[25000.0, 0.0, 0.0], [0.0, 25000.0, 0.0], [0.0, 0.0, 25000.0]])

#Old odometry
prevPose = None;

def publishState():
    global pub
    global mean
    global covar
    #Chuck mean and covar into state message and publish
    pub.publish(State(Header(), mean[0, 0], mean[1, 0], mean[2, 0], covar[0, 0], covar[1, 1], covar[2, 2]))

def actionUpdate(pose):
    global mean
    global covar

    if prevPose == None:
        prevPose = pose
        return

    #Calculate the change in pose, i.e. the action
    dx = pose.pose.pose.position.x - prevPose.pose.pose.position.x
    dy = pose.pose.pose.position.x - prevPose.pose.pose.position.y
    #Use the http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles, conversion, only 1 is non-zero
    dt = atan2((2 * pose.pose.pose.orientation.z * pose.pose.pose.orientation.w), (1 - 2 * (pose.pose.pose.orientation.z))) - atan2((2 * prevPose.prevPose.prevPose.orientation.z * prevPose.prevPose.prevPose.orientation.w), (1 - 2 * (prevPose.prevPose.prevPose.orientation.z)))

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

        publishState()
        
def kfilter():
    global pub
    pub = rospy.Publisher('State', State)
    rospy.init_node('localisation', anonymous=True)
    rospy.Subscriber('BeaconScan', Beacons, observationUpdate)
    rospy.Subscriber('Pose', PoseWithCovariance, actionUpdate)
    rospy.spin()

if __name__ == '__main__':
    kfilter()
