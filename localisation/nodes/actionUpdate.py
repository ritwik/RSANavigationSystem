#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from numpy import *
from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons
#from control.msg import Move
from std_msgs.msg import Header

def actionUpdate(data):
  p = data.position
  q = data.orientation
	# publish to the kfilter

def kfilter():
	global pub
	#    pub = rospy.Publisher('MOOOVE', State)
	rospy.init_node('actionUpdate', anonymous=True)
	rospy.Subscriber("pose", Move, actionUpdate)
	rospy.spin()

if __name__ == '__main__':
    kfilter()
