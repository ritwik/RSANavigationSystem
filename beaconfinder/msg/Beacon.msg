# Beacon message type, for a single beacon

# These IDs go from 0..n, for n beacons in increasing order of size
uint8 ID

# The x coordinate (global) of the centre of the beacon, according to the specification -- m
float32 x

# The y coordinate (global) of the centre of the beacon, according to the specification -- m
float32 y

# The distance (robot relative) of the centre of the beacon -- m
float32 distance

# The angle (robot relative) of the centre of the beacon -- radians
float32 angle

