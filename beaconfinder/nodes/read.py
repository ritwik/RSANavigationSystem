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
  #       b.append(Point())
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
  plan.append(Point(2,2))
  plan.append(Point(1,0))
  plan.append(Point(0,0))

  print "This plan has a value of "+str(heuristic(plan))
  drawPlan(plan)
# Plot Points

def plot(points):
  return 0

from math import sqrt
from math import *
def dist(point1,point2):
  return sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2)

def turns(point1,point2,angle):
  b = [10, 10]
  #print angle
  b[0] = atan2(point2.y-point1.y,point2.x-point1.x) - angle
  #print b[0]
  if b[0] >= pi:
    b[0] = 2*pi - b[0]
  if b[0] < 0:
    b[1] = -(b[0]-pi/2)
  else:
    b[1] = (b[0]-pi)
  b.sort()
  b.reverse()
  print b
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
    angle = turns(cur,plan[i+1],cur.a)[0] + cur.a

    # normalize the angle
    if angle < -pi:
      angle+=2*pi
    if angle > pi:
      angle-=2*pi
    print "Angle now "+str(angle)
    cur = plan[i+1]
    cur.a = angle
  return re

def drawPlan(plan):

  from matplotlib.pyplot import figure, show
  from matplotlib.patches import Ellipse
  import numpy as np

  if 1:
    fig = figure(1,figsize=(8,5))
    fig.clf()

    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-4,4), ylim=(-3,3))
    ax.plot([3, 3, -3, -3,3],[-2,2,2,-2,-2], 'r')

    els = [[-1,2],[3,0],[-1,-2]]
    r = 0.25
    for el in els:
      e = Ellipse((el[0], el[1]), r, r)
      e.set_color('g')
      r *= 1.5
      ax.add_patch(e)
    # TODO implement the radius properly

    # TODO plot an eclipse at the start (green)
    # plot ellipse at the end (red) # stop

    bx = fig.add_subplot(111, autoscale_on=False, xlim=(-4,4), ylim=(-3,3))
    #bx.scatter([.1,.1],[.2,.4])

    #x = 1
    #y = 2
    #dx = 0.5
    #dy = 0.5
    #bx.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1)
    cur = plan[0]
    bx.scatter(cur.x,cur.y,c='g')
    for i in range(len(plan)-1):

      # current angle
      angle = turns(cur,plan[i+1],cur.a)[0] + cur.a
      # normalize the angle
      if angle < -pi:
        angle+=2*pi
      if angle > pi:
        angle-=2*pi
      print angle


      # we go in a 'reverse' direction

      #plot the two points
      x = cur.x
      y = cur.y
      dx = plan[i+1].x-x
      dy = plan[i+1].y-y

      dx -= cos(atan2(dy,dx))*0.2
      dy -= sin(atan2(dy,dx))*0.2
      
      
      print str(angle)+"ANGLE"
      if (angle < 0 and cur.y < plan[i+1].y) or \
          (angle == 0 and cur.x > plan[i+1].y) or \
          (angle > 0 and cur.y > plan[i+1].y):
        print "REVERSE"
        bx.arrow( plan[i+1].x,plan[i+1].y, -dx, -dy, head_width=0.1, head_length=0.1)
      else:
        print "NORMAL"
        bx.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1)
      bx.scatter(cur.x,cur.y)

      cur = plan[i+1]
      cur.a = angle
    bx.scatter(cur.x,cur.y,c='r')

  show()

def okayState(state):
  # if the state is not good, then we return a high number
  # else return zero
  
  # if we cannot see any beacons in this state, it is not good !
  if ()
  
  return 0

if __name__ == '__main__':
  planner()
