cmake_minimum_required(VERSION 2.8.3)
project(ros_parking_management_msgs)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CapacityMsg.msg
  VehicleDimensionsMsg.msg
  VehicleInformationMsg.msg
  VehicleIdentificationMsg.msg
  ParkingGarageInformation.msg
  VehicleMotionInstructionMsg.msg
  VehicleAutonomousGradeMsg.msg
  VehicleTypeMsg.msg
  VehicleStatusMsg.msg
  VehicleLoadingMsg.msg
  ParkPreferencesMsg.msg
  TrajectoryMsg.msg
  Car2xVehicleState.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CapacityRequest.srv
  RegisterVehicleRequest.srv
  VehiclePositionRequest.srv
  ParkoutVehicleRequest.srv
  AddVehicleRequest.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#LIBRARIES
CATKIN_DEPENDS
  roscpp
  rospy
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)