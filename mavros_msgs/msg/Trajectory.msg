# MAVLink message: TRAJECTORY
# http://mavlink.org/messages/common#TRAJECTORY

std_msgs/Header header

uint8 type # See enum MAV_TRAJECTORY_REPRESENTATION.
uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0
uint8 MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1

float32[11] point_1
float32[11] point_2
float32[11] point_3
float32[11] point_4
float32[11] point_5

uint8[5] point_valid # States if respective point is valid.
