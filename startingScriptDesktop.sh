#!/bin/sh
ROBOT=bass

gnome-terminal --command="bash -c \"roscore; exec bash\"" &
gnome-terminal --command="bash -c \"rosrun p2os_dashboard p2os_dashboard; exec bash\"" &

scp startingScriptRobot.sh $ROBOT:~/.
gnome-terminal --command="bash -c \"ssh $ROBOT \"./startingScriptRobot.sh\"\"" #Source bash_rc in this script

gnome-terminal --command="bash -c \"rosrun beaconfinder beaconfinder.py\"" &
gnome-terminal --command="bash -c \"rosrun localisation kfilter.py\"" &
gnome-terminal --command="bash -c \"rosrun control control.py\"" &
