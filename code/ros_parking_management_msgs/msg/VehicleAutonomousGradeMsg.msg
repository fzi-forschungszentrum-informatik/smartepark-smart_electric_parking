###
#
# When signing in, the vehicle needs to announce its autonomous grade. 3 different flags are available.
# If a car has a high autonomous grade, the PMS only assigns a parking space.
# Otherwise the PMS takes over the missing functions.
#
###

uint16 autonomous_grade
uint16 autonomous_grade_none = 0
uint16 autonomous_grade_actuators = 2
uint16 autonomous_grade_fully_automated = 5
