#!/usr/bin/env python
import roslib; roslib.load_manifest('localisation')
import rospy

from math import *

from tf.transformations import euler_from_quaternion

q2 = 0.99
q3 = 0.15
phi = atan2((2 * (0 * 0 + q2 * q3)), (1 - 2 * (q2 ** 2)))
print phi
print phi * 180 / pi
#theta = asin(2 * (0 * 0.64 - (-0.76) * 0))
#psi = atan(2 * (0 * -0.76 + 0 * 0.64)/
