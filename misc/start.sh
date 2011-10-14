#!/bin/bash
ROBOT=wills
MAIN="main main.py"
IP=`ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}' | head -1`
roscore &

variable=$(cat <<SETVAR
#!/bin/bash
# . /opt/ros/diamondback/setup.bash
# . ~/.bashrc 
# exec /opt/ros/diamondback/setup.bash
# sh /opt/ros/diamondback/setup.bash
source /opt/ros/diamondback/setup.bash 
export ROS_PACKAGE_PATH=~/ros_workspace:\$ROS_PACKAGE_PATH

# CLIENT_IP="${SSH_CLIENT%% *}"
CLIENT_IP="${IP}"
export ROS_MASTER_URI="http://\${CLIENT_IP}:11311/" 

exec 3>/dev/ttyS2
rosrun p2os_driver p2os &
rosparam set sicklms/port /dev/ttyS2
rosparam set sicklms/baud 38400
rosrun beaconfinder beaconfinder.py
SETVAR
)
echo "$variable"
echo "$variable" | ssh $ROBOT "cat > ~/script.sh"
# ssh $ROBOT "sh ~/script.sh"
ssh $ROBOT "chmod +x script.sh;~/script.sh"

# We have to wait a while to get the robot to start up

rosrun p2os_dashboard p2os_dashboard & # keep one up just in case

rosrun pull...

rosrun beaconfinder beaconfinder.py

rosrun localisation kfilter.py

# wait a while then run the controller and pathfinder

# should we add a few other things like

# rosrun ${MAIN}

# use roslaunch
# 
