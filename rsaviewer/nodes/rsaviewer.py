#!/usr/bin/env python
import roslib; roslib.load_manifest('rsaviewer')
import rospy
import threading
#from numpy import *
#from math import *

from localisation.msg import State
from beaconfinder.msg import Beacons
from sensor_msgs.msg import LaserScan
#from std_msgs.msg import Header
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion

from Tkinter import *
from math import sin,cos,pi
seenBeacons = []
drawnBeacons = []
laserCanvasItems = []
#def drawRobot(c):
	
global canvas
PIXELSPERMETER = 100
offset = [400,400]
realBeacons = [[0,-1,2,0.052],[1,-1,-2,0.138],[2,3,0,0.223]]
robotPose = [0,0,0]
robotCovar = [1,1,1]
robotCanvasItems = []
def worldToCanvas(x,y):
	'''Converts an xy pair to an xy on the canvas, assuming x & y are in pixels
	'''
	return [x+offset[0], (y*(-1)+offset[1])]

def mWorldToCanvas(x,y):
	return worldToCanvas(mToPixels(x),mToPixels(y))

def mToPixels(x):
	return x*PIXELSPERMETER


def drawGrid():
	canvas.create_line(0, offset[1], offset[0]*2, offset[1])
        canvas.create_line(offset[0], 0, offset[0],offset[1]*2)
	
	for i in range(1,4):
		#horizontal lines above y=0
		[x0,y0] = mWorldToCanvas(-5,i)
		[x1,y1] = mWorldToCanvas(5,i)
		canvas.create_line(x0,y0,x1,y1,fill="grey")
		#horizontal lines below y=0
		[x0,y0] = mWorldToCanvas(-5,-i)
		[x1,y1] = mWorldToCanvas(5,-i)
		canvas.create_line(x0,y0,x1,y1,fill="grey")

		#vertical lines above x=0
		[x0,y0] = mWorldToCanvas(i,-5)
		[x1,y1] = mWorldToCanvas(i,5)
		canvas.create_line(x0,y0,x1,y1,fill="grey")

		#vertical lines below x=0
		[x0,y0] = mWorldToCanvas(-i,-5)
		[x1,y1] = mWorldToCanvas(-i,5)
		canvas.create_line(x0,y0,x1,y1,fill="grey")


def drawRealBeacons():
        global canvas,offset,beaconRadii,realBeacons
        for beacon in realBeacons:
                r = mToPixels(beacon[3])
                [xc,yc] = worldToCanvas(mToPixels(beacon[1]),mToPixels(beacon[2]))
                print xc,yc,beacon[1],beacon[2]
                bbox = xc-r,yc-r,xc+r,yc+r
                #bbox = offset[0]+beacon[1]*100-beacon[3]/10.0, offset[1]+beacon[2]*100-beacon[3]/10.0, offset[0]+beacon[1]*100+beacon[3]/10.0, offset[1]+beacon[2]*100+beacon[3]/10.0
                print "real beacon",beacon,bbox
                newBeacon = canvas.create_oval(bbox, fill="blue")
                #drawnBeacons.append(newBeacon)


def drawLaserPoints(data):
	global robotPose,laserCanvasItems
	[rx,ry,rHeading] = robotPose
	angleIncrement = data.angle_increment
	startAngle = data.angle_min
	endAngle = data.angle_max
	for laserPoint in laserCanvasItems:
		canvas.delete(laserPoint)
	#enumDataRange = enumerate(data.ranges)
	#draw lines to the first points
	#[iToFirstPoint,dTofirstPoint] = enumDataRange[0]
	#[iToLastPoint,dToLastPoint] = enumDataRange[len(enumDataRange)-1]
	#xf = cos(rHeadingTheta+startAngle)/dToFirstPoint
	#yf = sin(rHeadingTheta+startAngle)/dToFirstPoint
	#xl =  cos(rHeadingTheta+endAngle)/dToFirstPoint
	#yl =  sin(rHeadingTheta+endAngle)/dToFirstPoint
	#laserCanvasItems.append(canvas.create_line(rx,ry,rx+xf,ry+yf,fill="green"))
	#print "drawing line to laser: ",rx,ry,rx+xf,ry+yf
	currentAngle = startAngle
	for index,distance in enumerate(data.ranges):
		#distance = mToPixels(distance)
		pX =  cos(rHeading+(startAngle+index*angleIncrement))*distance
		pY  = sin(rHeading+(startAngle+index*angleIncrement))*distance	
		#laserCanvasItems.append(canvas.create_line(mToPixels(rx),mToPixels(ry),mToPixels(pX),mToPixels(pY),fill="green"))
		#print "drawing at ", mToPixels(rx),mToPixels(ry),mToPixels(pX),mToPixels(pY)
		#if (distance < DISTANCE_THRESHOLD):
		##	pointx = distance * math.cos(startAngle+ index*angleIncrement) 
		#	pointy = distance * math.sin(startAngle+ index*angleIncrement)
		#	if (abs(distance-prevDistance) > BEACON_JUMP_THRESHOLD):
		#		#close the older pillar if there was one
		#		if (len(currentPillar) > 0):
		#			potentialPillars.append(currentPillar)
		#			currentPillar = []
		#	currentPillar.append([pointx,pointy,distance])
		#else:
		#	if (len(currentPillar) > 0):
		#		potentialPillars.append(currentPillar)
		#		currentPillar = []
		#prevDistance = distance	

def drawBeacons(data):
	global seenBeacons,drawnBeacons,canvas,offset,robotPose
	[rx,ry,rHeading] = robotPose
	#canvas.create_line(0, 100, 200, 0, fill="red", dash=(4, 4))
	for drawnBeacon in drawnBeacons:
		canvas.delete(drawnBeacon)
	for beacon in data.beacon:
		r = mToPixels(realBeacons[beacon.ID][3])
		#bbox = offset[0]+beacon.x*100-r, offset[1]+beacon.y*100-r, offset[0]+beacon.x*100+r, offset[1]+beacon.y*100+r
		pX =  cos(rHeading+beacon.angle)*beacon.distance
		pY  = sin(rHeading+beacon.angle)*beacon.distance	
		#xc = mToPixels(pX)
		#yc = -mToPixels(pY)
		[xc,yc] = mWorldToCanvas(rx+pX,ry+pY)
		#[xc,yc] = worldToCanvas(mToPixels(beacon.x),mToPixels(beacon.y))
		bbox = xc-r, yc-r, xc+r, yc+r
		newBeacon = canvas.create_oval(bbox, fill="",outline="red")
		drawnBeacons.append(newBeacon)
		print beacon
		print "drawn at ",bbox
		#print "drawn Beacon"

	

def updateBeacons(beacons):
	global seenBeacons
	seenBeacons = beacons.beacon
	print beacons

def updateRobotLocation(state):
	global robotCanvasItems,robotPose,robotCovar
	for item in robotCanvasItems:
		canvas.delete(item)
	r = mToPixels(0.2)
	robotPose = [state.x,state.y,state.theta]
	print "robot is at",robotPose 
	robotCovar = [state.xCovar,state.yCovar,state.thetaCovar]
	#bbox = state.x+r,state.y+r,state.x-r,state.y-r
	px = cos(state.theta)*0.2
	py = -sin(state.theta)*0.2
	print "calculated px,py",px,py
	px = mToPixels(px)
	py = mToPixels(py)
	[rx,ry] = mWorldToCanvas(state.x,state.y)
	print "new px,py,rx,ry,rx+px,ry+py",px,py,rx,ry,rx+px,ry+py
	bbox = rx+r,ry+r,rx-r,ry-r
	robotCanvasItems.append(canvas.create_oval(bbox,fill="red"))
	robotCanvasItems.append(canvas.create_line(rx,ry,rx+px,ry+py,fill="black"))
	
	#draw the covariances
	covx= mToPixels(robotCovar[0])
	covy= -mToPixels(robotCovar[1])
	covarbox = rx-covx,ry-covy,rx+covx,ry+covy
	print "robot bbox",bbox
	print "covar bbox",covarbox
	robotCanvasItems.append(canvas.create_oval(covarbox,fill="",outline="green"))

def RSAviewer():
	#rsawindow = TkVisualisation()
	#r = RSADraw()
	global canvas
	root=Tk()
	root.protocol("WM_DELETE_WINDOW", root.quit())
	frame = Frame(root)
	frame.pack()
	canvas = Canvas(root, width = 800, height=800)
	canvas.pack()
	button = Button(frame, text="QUIT", fg="red", command=frame.quit)
	button.pack(side=LEFT)
	#canvas.create_line(0, offset[1], offset[0]*2, offset[1])
	#canvas.create_line(offset[0], 0, offset[0],offset[1]*2)
	drawGrid()
	drawRealBeacons()
	rospy.init_node('rsaviewer', anonymous=True)
	rospy.Subscriber("base_scan", LaserScan, drawLaserPoints)
	rospy.Subscriber('BeaconScan', Beacons, drawBeacons)
	rospy.Subscriber('State', State, updateRobotLocation)
	root.mainloop()
	
	

if __name__=="__main__":
	RSAviewer()

