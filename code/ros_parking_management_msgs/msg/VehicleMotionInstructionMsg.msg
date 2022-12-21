###
#
# Information sent from the PMS to the vehicle when the vehicle is parking and unparking.
#
###

std_msgs/Header header

# id of the Vehicle that this message is for
int16 id

# current pose
geometry_msgs/Pose2D current_pose

VehicleStatusMsg status

## High level instructions
int32[] route
int32 destination_parking_space_id

TrajectoryMsg trajectory