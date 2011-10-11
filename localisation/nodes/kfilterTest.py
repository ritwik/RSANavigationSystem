#!/usr/bin/python

from numpy import *
from math import *



mean = array([[0.0], [0.0], [0.0]])
covar = array([[25000.0, 0.0, 0.0], [0.0, 25000.0, 0.0], [0.0, 0.0, 25000.0]])

def observationUpdate(dx, dy, distance, angle):
    global mean
    global covar

    #Set up necessary matrices
    #Something tells me this is very wrong atm
    H = array([[0, 0, 0],
               [dx / sqrt(dx ** 2 + dy ** 2), dy / sqrt(dx ** 2 + dy ** 2), 0],
               [dy / (dx ** 2 + dy ** 2), -dx / (dx ** 2 + dy ** 2), -1]])

    print "H = " + str(H)

    print "Hx = " + str(dot(H, mean))

    R = array([[1, 1, 1],
               [0, 0.03 * distance, 0],
               [0, 0, 0.1 * abs(angle)]]) #Should these be squared?!

    print "R = " + str(R)

    #Chuck beacon observation into matrix
    z = array([[0], [distance], [angle]])

    print "z = " + str(z)
    print angle * 180 / pi

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

for i in range(0, 10):
    observationUpdate(-1.0, 2.0, 5 ** 0.5, atan2(2.0, 1.0))
    observationUpdate(-1.0, -2.0, 5 ** 0.5, atan2(-2.0, 1.0))
    #This works!
