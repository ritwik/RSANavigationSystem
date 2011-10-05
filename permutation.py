#!/usr/bin/env python
from itertools import permutations
import random
list = []
class Strange:
  def __init__(self,x=0,y=1):
    self.x = random.randint(1,100)
    self.y = random.randint(1,100)
  def __str__(self):
    return "OK"
    return format("x{0} y{1}",self.x,self.y)


for i in range(4):
  list.append(Strange())


for i in range(len(list)):
  print list[i].x+list[i].y
  
for i in permutations(list):
  for n in i:
    print "x{0} y{1} ".format(n.x,n.y),
    
    
# the plan is to detect all the bad nodes, and then remove them from all
# the permutations to make things easier

# bad plans are lists of two elements