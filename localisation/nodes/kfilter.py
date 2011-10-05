from numpy import *
from math import *

pub = rospy.Publisher('State', State)

theta = 0.0
x = 0.0
y = 0.0

def actionUpdate(move):
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
               [0, 0, 1]])

    Q = R * E * transpose(R)
    
    #And now all of the formulae
    
    
def observationUpdate(beacons):
    
    
    


def kfilter():
    rospy.Subscriber("BeaconScan", Beacons, observationUpdate)
    rospy.Subscriber("Movement", Movements, actionUpdate)
    rospy.spin()

