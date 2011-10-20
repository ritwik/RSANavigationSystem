#!/usr/bin/env python
import roslib; roslib.load_manifest('rsaviewer')
import rospy
import threading
#from numpy import *
#from math import *

#from localisation.msg import State
from beaconfinder.msg import Beacons
#from std_msgs.msg import Header
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion

from Tkinter import *

seenBeacons = []
drawnBeacons = []

#def drawRobot(c):
	
global canvas
offset = [400,400]
realBeacons = [[0,-1,2,52],[1,-1,-2,138],[2,3,0,223]]
def drawBeacons(data):
	global seenBeacons,drawnBeacons,canvas,offset
	
	#canvas.create_line(0, 100, 200, 0, fill="red", dash=(4, 4))
	for drawnBeacon in drawnBeacons:
		canvas.delete(drawnBeacon)
	for beacon in data.beacon:
		r = realBeacons[beacon.ID][3]/10.0
		bbox = offset[0]+beacon.x*100-r, offset[1]+beacon.y*100-r, offset[0]+beacon.x*100+r, offset[1]+beacon.y*100+r
		newBeacon = canvas.create_oval(bbox, fill="red")
		drawnBeacons.append(newBeacon)
		print beacon
		print "drawn at ",bbox
		#print "drawn Beacon"
		
def drawRealBeacons():
	global canvas,offset,beaconRadii,realBeacons
	for beacon in realBeacons:
		bbox = offset[0]+beacon[1]*100-beacon[3]/10.0, offset[1]+beacon[2]*100-beacon[3]/10.0, offset[0]+beacon[1]*100+beacon[3]/10.0, offset[1]+beacon[2]*100+beacon[3]/10.0
		print "real beacon",beacon,bbox
		newBeacon = canvas.create_oval(bbox, fill="blue")
		drawnBeacons.append(newBeacon)

def updateBeacons(beacons):
	global seenBeacons
	seenBeacons = beacons.beacon
	print beacons

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
	canvas.create_line(0, offset[1], offset[0]*2, offset[1])
	canvas.create_line(offset[0], 0, offset[0],offset[1]*2)
	drawRealBeacons()
	rospy.init_node('rsaviewer', anonymous=True)
	#rospy.Subscriber("scan", LaserScan, callback)
	rospy.Subscriber('BeaconScan', Beacons, drawBeacons)
	root.mainloop()
	
	

if __name__=="__main__":
	RSAviewer()

