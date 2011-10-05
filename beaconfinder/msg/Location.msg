# Location Variable, I'm not sure whether I need this
# please comment, if it's not so good

uint8 id # just to help debug, eg id 1-5 location of targets
# and 0 is the location of the robot

# The x coordinate (robot relative) of the centre of the beacon
float32 x

# The y coordinate (robot relative) of the centre of the beacon
float32 y

# the current orientation of the robot
float32 a # measured in radians

bool isForward # forward is defined by the robot having an orientation 
# facing the node to be travelled too
#
# this needs help with
#
#

# this may be super messy but
float32 dist
# dist is measured in meters
# negative is backward, positive forward

float32 turn
# turn is radians to turn... =)
