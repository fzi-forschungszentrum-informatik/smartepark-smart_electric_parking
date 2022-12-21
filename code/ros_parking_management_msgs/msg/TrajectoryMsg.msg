###
#
# The trajectory message consists of arrays of discrete poses and velocitys. 
# Poses are not georeferenced. Instead they are the defined coordinate system.
#
###

uint8 vehicle_id

uint32 count
time[] ros_time

## Detailled low level instructions
# time elasped since header time
float32[] t


# coordinates and orientation in a 2D frame defined in the header
float32[] x
float32[] y
float32[] yaw

# velocity in driving direction
float32[] v # negative velocity is reverse