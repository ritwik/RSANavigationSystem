#!/usr/bin/env python

POINTS_FILE = "points.cfg" # case sensitive

class Point:
	def __init__(self, x=0, y=0, a=0):
		self.x = x
		self.y = y
		self.a = a

class Node:
	def __init__(self, x, y, f):
		self.x = x
		self.y = y
		self.f = f # 1 for forward, else we're going in 
		# the negative spin (longer direction)



def planner():
	b = []
	# for x in range(50):	
	# 	b.append(Point())
	print "OKAY"

	f = open(POINTS_FILE,'r') #r is assumed
	for line in f:
		p = line.split()
		x = float(p[0])
		y = float(p[1])
		print str(x)+","+str(y)
		b.append(Point(x,y))
		print len(b)
	f.close()
	plan = []
	plan.append(Point(0,0)) # starting position
	plan.append(Point(1,1))
	plan.append(Point(1,1))
	plan.append(Point(1,0))
	plan.append(Point(2.185039863261519,1))

	print "This plan has a value of "+str(heuristic(plan))

# Plot Points

def plot(points):
	return 0		
	
from math import sqrt
from math import *
def dist(point1,point2):
	return sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2)

def turns(point1,point2,angle):
	b = [10, 10]
	print angle
	b[0] = atan2(point2.y-point1.y,point2.x-point1.x) - angle
	print b[0]
	if b[0] >= pi:
		b[0] = 2*pi - b[0]
	if b[0] < 0:
		b[1] = -(b[0]-pi/2)
	else:
		b[1] = (b[0]-pi)
	b.sort()
	# print b
	return b

def heuristic(plan):
	re = 0
	cur = plan[0]
	curAngle = plan[0].a
	for i in range(len(plan)-1):
		re += dist(cur,plan[i+1])*5
		
		# find the angle we need to rotate
		# format
		print 'Currently Facing: {theta}, Required Turn {req}'.format(theta=cur.a, req=turns(cur,plan[i+1],cur.a)[0])

		# current angle
		angle = turns(cur,plan[i+1],cur.a) 
		print "Angel now "+str(angle)
		cur = plan[i+1]
		cur.a = angle[0] + cur.a
	return re

if __name__ == '__main__':
	planner()
