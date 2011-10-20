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

class TkVisualisation(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.start()
	def callback(self):
		self.root.quit()
	def run(self):
		self.root=Tk()
		self.root.protocol("WM_DELETE_WINDOW", self.callback)
		self.s = StringVar()
		self.s.set('Foo')
		l = Label(self.root,textvariable=self.s)
		l.pack()
		#w = Label(root, text="")
		#w.pack()
		frame = Frame(self.root)
        	frame.pack()
		canvas = Canvas(self.root, width = 600, height=600)
		canvas.pack()
		#drawRobot(canvas)
		#drawRealBeacons(canvas)
		drawBeacons(canvas)
		#drawLaser(canvas)
        	self.button = Button(frame, text="QUIT", fg="red", command=frame.quit)
        	self.button.pack(side=LEFT)
		self.root.mainloop()



#def drawRobot(c):
	

def drawBeacons(c):
	global seenBeacons,drawnBeacons
	c.create_line(0, 0, 200, 100)
	c.create_line(0, 100, 200, 0, fill="red", dash=(4, 4))
	for drawnBeacon in drawnBeacons:
		c.delete(drawnBeacon)
	for beacon in seenBeacons:
		bbox = beacon.x-30, beacon.y-30, beacon.x+30, beacon.y+30
		create_oval(bbox, fill="red")

def updateBeacons(beacons):
	global seenBeacons
	seenBeacons = beacons.beacon

def RSAviewer():
	rsawindow = TkVisualisation()

	rospy.init_node('rsaviewer', anonymous=True)
	#rospy.Subscriber("scan", LaserScan, callback)
	rospy.Subscriber('BeaconScan', Beacons, updateBeacons)
	rospy.spin()

if __name__=="__main__":
	RSAviewer()

