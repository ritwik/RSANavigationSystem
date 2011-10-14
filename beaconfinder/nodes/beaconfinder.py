#!/usr/bin/env python
import roslib; roslib.load_manifest('beaconfinder')
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from beaconfinder.msg import Beacons,Beacon

import random
#from numpy import sqrt

#All these constants are in mm where they refer to a distance

#This sets the maximum distance of a scan point to consider
DISTANCE_THRESHOLD = 5000 

#This is the distance difference between 1 point and the next on a scan that is considered an 'edge', i.e. new beacon
BEACON_JUMP_THRESHOLD = 75

#This is the accepted error from the known beacon radius that we accept, i.e, a circle we find is beacon A, if circle.radius is A.radius+/- BEACON_MATCH_THRESHOLD
BEACON_MATCH_THRESHOLD = 30

#The max acceptable error from a consensus set to it's predicted circle. This lets us ignore walls. Smaller values = only accept more distinct circles
ERROR_THRESHOLD = 100
pub = None

realBeacons = [[0,-1,2,52],[1,-1,-2,138],[2,3,0,223]]

def getCircleFrom(points):
	((x1,y1),(x2,y2),(x3,y3)) = points
	#print "x1 = " + str(x1) + " y1 = " + str(y1)
	#print "x2 = " + str(x2) + " y2 = " + str(y2)
	#print "x3 = " + str(x3) + " y3 = " + str(y3)
	ma = (y2-y1)/(x2-x1)
	mb = (y3-y2)/(x3-x2)
	xc = ( ma*mb*(y1-y3) + mb *(x1+x2) - ma*(x2 + x3) )/ ( 2.0*(mb-ma) )
	#yc = (-1/ma) * (xc - (x1+x2)/2) + (y1+y2)/2
	# Why does ^ not work? yet the following works? WTF?
	yc = (-1/mb) * (xc - (x2+x3)/2) + (y2+y3)/2
	r = math.sqrt((x1-xc)**2 + (y1-yc)**2)
	#print "ma = "+ str(ma) + " mb = " + str(mb)
	#print "xc = "+ str(xc) + " yc = " + str(yc)
	#print "r = "+ str(r) 
	return [xc,yc,r]
	
def getErrorFromCircle(point, circle):
	[x,y] = point
	[xc,yc,r] = circle
	return (math.sqrt((y-yc)**2 + (x-xc)**2) - r)

def getPillarFrom(potentialPoints,k,t,d):
	''' Uses RANSAC to find a circle in the data points sent
	'''
	#foundPillars = [[2,3,12]]
	iterations = 0
	bestCircle = []
	bestConsensus_set = []
	bestError = ERROR_THRESHOLD
	
	while iterations < k :
		possibleInliers = random.sample(potentialPoints, 3)
		possibleCircle = getCircleFrom(possibleInliers)
		consensusSet = []
		circleError = 0
		#Find the consensus set (points which general fit this model)
		for point in potentialPoints:
			#get error of point from model
			error = getErrorFromCircle(point, possibleCircle)
			#print error
			#if abs value of error is less than t% of r, add to consensus set
			if (abs(error) < t*possibleCircle[2]):
				circleError = circleError + error**2
				consensusSet.append(point)
		
		if (iterations == 0):
			bestCircle =  possibleCircle
			bestConsensusSet = consensusSet
			bestError = circleError
		
		
		if (len(consensusSet) >= d) and (circleError < bestError):
			#this is now the best model
			bestCircle =  possibleCircle
			bestConsensusSet = consensusSet
			bestError = circleError
			#print "Best error is ",bestError
		
		iterations = iterations +1	
	
	[x,y,r] = bestCircle
	if (bestError < ERROR_THRESHOLD):
		#print "Keeping pillar: ",potentialPoints[0], " to ",potentialPoints[-1], "with radius: ",r, " and error: ",bestError
		return bestCircle
	else: 
		#print "Dropping pillar: ",potentialPoints[0], " to ",potentialPoints[-1]
		return [0,0,0]


def generateBeaconList(pillars):
	'''Returns a list of beacons sorted by size, labelled by ID (currently assumes smallest one is id 0)
	*might change this later to actually compare to real beacon sizes and estimate which one each pillar is*
	'''
	beacons = []
	sortedPillars =  sorted(pillars, key=lambda pillar: pillar[2])
	for pillar in sortedPillars:
		[x,y,r] = pillar
		distance = math.sqrt(x**2 + y**2)
		angle = math.atan2(y,x)
		for realBeacon in realBeacons:
			[bid,bx,by,br] = realBeacon
			if abs(br - r) < BEACON_MATCH_THRESHOLD:
				print "pillar: ",x,", ",y,", ",r," matches beacon ",bid," Keeping"
				beacons.append(Beacon(bid,bx,by,distance/1000,angle))
				break
			else:
				print "pillar: ",x,", ",y,", ",r," does not match beacon ",bid
	return beacons
		
def callback(data):
	# rospy.loginfo("\n Laser read: \n")
	# rospy.loginfo(data.header)
	# rospy.loginfo(data.ranges)
	potentialPillars = []
	currentPillar = []
	angleIncrement = data.angle_increment
	startAngle = data.angle_min
	prevDistance = DISTANCE_THRESHOLD
	for index,distance in enumerate(data.ranges):
		distance = distance * 1000
		if (distance < DISTANCE_THRESHOLD):
			pointx = distance * math.cos(startAngle+ index*angleIncrement) 
			pointy = distance * math.sin(startAngle+ index*angleIncrement)
			if (abs(distance-prevDistance) > BEACON_JUMP_THRESHOLD):
				#close the older pillar if there was one
				if (len(currentPillar) > 0):
					potentialPillars.append(currentPillar)
					currentPillar = []
			currentPillar.append([pointx,pointy])
		else:
			if (len(currentPillar) > 0):
				potentialPillars.append(currentPillar)
				currentPillar = []
		prevDistance = distance	
	rospy.loginfo("Found %d potential pillars", len(potentialPillars));
	pillars = []
	beacons = []
	for potentialPillar in potentialPillars:
		#rospy.loginfo("Looking at a cloud of %d points, [%.2lf,\t %.2lf \t\ts = %.2lf])", len(potentialPillar),
		if len(potentialPillar)>=3:
			pillar = getPillarFrom(potentialPillar,20,0.20,5)
			[x,y,r] = pillar
			if r > 0:	
				pillars.append(pillar)
				rospy.loginfo("Looking at a cloud of %d points, found: [%.2lf,\t %.2lf \t %.2lf])", len(potentialPillar), pillar[0],pillar[1],pillar[2]) 
	beacons = generateBeaconList(pillars)
	rospy.loginfo("Found %d beacons", len(beacons))
	print beacons
	 
	#pillars = getPillarsFrom(potentialPillars)
	#for [x,y,radius] in pillars:
		#rospy.loginfo("[%.2lf,\t %.2lf \t\ts = %.2lf]",x,y,radius)
	#[x,y,r] = getCircleFrom([-1,0],[0,1],[1,0])
	#rospy.loginfo("Circle = %lf, %lf, %lf",x,y,r)
	global pub
	pub.publish(Beacons(Header(),len(beacons),beacons))
	

def beaconfinder():
	global pub
	pub = rospy.Publisher('BeaconScan', Beacons)
	rospy.init_node('beaconfinder', anonymous=True)
	rospy.Subscriber("scan", LaserScan, callback)
	rospy.spin()

if __name__ == '__main__':
	beaconfinder()
