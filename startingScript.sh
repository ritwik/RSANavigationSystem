#!/bin/sh
MAIN=beaconfinder
ROBOT=bass

roscore &
rosrun p2os_dashboard p2os_dashboard & # keep one up just in case

# set ROS_MASTER_URI on robot, typically by sshing in?

ssh $ROBOT "rosrun p2os_driver p2os" # Do I need to initialize bash first?
ssh $ROBOT "rosrun p2os_teleop p2os_teleop" # I think I can do this on the 
# desktop

# rosrun p2os_dashboard p2os_dashboard
# we don't want a dashboard so we'll probably just tell the robot to start



# starting
rosrun ${MAIN} start.py # maybe this should wait until the robot finishes running

rosrun ${MAIN} main.py # Is this good? can this still catch input?


rosrun $MAIN stop.py # disable movement of the robot
