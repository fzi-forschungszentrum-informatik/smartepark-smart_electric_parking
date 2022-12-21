###
#
# This message communicates the current state of a vehicle to the PMS.
# Additionally to the ID, Position and velocity, the battery level is communicated if the car is electric. 
#
###


# Information have to be in global map frame
std_msgs/Header header

# identifier or station id
int32 station_id
int32 user_id

# coordinates and orientation in the 2D map frame
float32 x
float32 y
float32 yaw

# velocity in driving direction
float32 v

# encoded status information
uint16 status

# flag to indicate if the vehicle has completed its motion
bool in_motion

# charging status of the vehicle
uint8 battery_level
