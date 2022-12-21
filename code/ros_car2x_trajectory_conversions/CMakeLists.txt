cmake_minimum_required(VERSION 2.8.12)
project(ros_car2x_trajectory_conversions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  tf2
  eigen_conversions

  fzi_traffic_planning_msgs
  ros_parking_management_msgs

  ros_geom
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(geom REQUIRED)
find_package(math REQUIRED)
find_package(traj REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES ${PROJECT_NAME}_base
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    fzi_traffic_planning_msgs
    geometry_msgs
    visualization_msgs
    tf
    tf2
  DEPENDS
    EIGEN3
    math
    geom
    traj
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# -----------------------------------------------
# _car2x_to_trajectory_converter
# -----------------------------------------------
add_executable(${PROJECT_NAME}_car2x_to_trajectory_converter
  src/car2x_to_trajectory_converter.cpp
)

set_target_properties(${PROJECT_NAME}_car2x_to_trajectory_converter PROPERTIES OUTPUT_NAME car2x_to_trajectory_converter PREFIX "")

#add_dependencies(${PROJECT_NAME}_car2x_to_trajectory_converter
#)

target_link_libraries(${PROJECT_NAME}_car2x_to_trajectory_converter
  ${catkin_LIBRARIES}
  geom::geom
  traj::traj
  math::math
)

install(TARGETS
  ${PROJECT_NAME}_car2x_to_trajectory_converter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)