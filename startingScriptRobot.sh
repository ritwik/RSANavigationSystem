source /opt/ros/diamondback/setup.bash

CLIENT_IP="${SSH_CLIENT%% *}"

export ROS_MASTER_URI="http://${CLIENT_IP}:11311/"

rosparam set sicklms/port /dev/ttyS2
rosparam set sicklms/baud 38400

#Using & for now, assuming they get killed when ssh is killed
bash -c "rosrun sicktoolbox_wrapper sicklms" &
#bash -c "rosrun p2os_driver p2os" &

