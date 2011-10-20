#!/bin/sh

gnome-terminal --command="bash -c \"roscore; exec bash\"" &

gnome-terminal --command="bash -c \"rosrun beaconfinder beaconfinder.py\"" &
gnome-terminal --command="bash -c \"rosrun localisation kfilter.py\"" &
gnome-terminal --command="bash -c \"rosrun stage stageros `rospack find control`/beacons.world\"" &
gnome-terminal --command="bash -c \"rosrun control control.py\"" &

