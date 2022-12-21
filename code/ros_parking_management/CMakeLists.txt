cmake_minimum_required(VERSION 3.5.1)
project(ros_parking_management)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf2_ros
  tf2_geometry_msgs

  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_traffic_rules
  lanelet2_matching
  lanelet2_routing
  lanelet2_python

  fzi_sensor_msgs
  fzi_car2x_msgs
  ros_parking_management_msgs
  fzi_curvepoint_msgs

  ros_lanelet2_visualization
  ros_scene_prediction
  ros_scene_prediction_msgs
  ros_geom
  utils
  steering_functions
)

## System dependencies are found with CMake's conventions
find_package(Boost 1.65 REQUIRED COMPONENTS system serialization thread date_time)

find_package(math)
find_package(math_openmp)
find_package(geom)
find_package(gprj)
find_package(colors)
find_package(traj)
find_package(fltr)

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
else()
  message(STATUS "OPENMP NOT FOUND")
endif()

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
  INCLUDE_DIRS include
  LIBRARIES ros_parking_management
  CATKIN_DEPENDS
    steering_functions
    ros_scene_prediction
    roscpp
    rospy
    std_msgs
    lanelet2_core
    lanelet2_io
    lanelet2_projection
    lanelet2_routing
    lanelet2_traffic_rules
    ros_parking_management_msgs
  DEPENDS boost system_lib math geom gprj colors fltr OpenMP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ros_parking_management

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Vehicle.cpp
  src/${PROJECT_NAME}/ParkingScene.cpp
  src/${PROJECT_NAME}/ParkingSpace.cpp
  src/${PROJECT_NAME}/ParkingAssignment.cpp
  src/${PROJECT_NAME}/ParkingManagementSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  math::math
  math::math_openmp
  geom::geom
  colors::colors
  traj::traj
  fltr::fltr
  ${OpenMP_CXX_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OpenMP_FLAGS})

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# vehicle_handler_node

add_executable(${PROJECT_NAME}_vehicle_handler_node
  src/vehicle_handler_node.cpp
)

add_dependencies(${PROJECT_NAME}_vehicle_handler_node
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_vehicle_handler_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  math::math
  math::math_openmp
  geom::geom
  fltr::fltr
  ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_vehicle_handler_node PROPERTIES OUTPUT_NAME vehicle_handler_node PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_vehicle_handler_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# parking_management_system_node

add_executable(${PROJECT_NAME}_parking_management_system_node
  src/parking_management_system_node.cpp
)

target_link_libraries(${PROJECT_NAME}_parking_management_system_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  math::math
  math::math_openmp
  geom::geom
  gprj::gprj
  fltr::fltr
)

add_dependencies(${PROJECT_NAME}_parking_management_system_node
  ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_parking_management_system_node PROPERTIES OUTPUT_NAME parking_management_system_node PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_parking_management_system_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# demo_vehicle_side

add_executable(${PROJECT_NAME}_demo_vehicle_side
  src/demo_vehicle_side.cpp
)

add_dependencies(${PROJECT_NAME}_demo_vehicle_side
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_demo_vehicle_side
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  math::math
  math::math_openmp
  geom::geom
  fltr::fltr
  ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_demo_vehicle_side PROPERTIES OUTPUT_NAME demo_vehicle_side PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_demo_vehicle_side
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## demo_infrastructure_side_vehicle_components

add_executable(${PROJECT_NAME}_demo_infrastructure_side_vehicle_components
  src/demo_infrastructure_side_vehicle_components.cpp
)

add_dependencies(${PROJECT_NAME}_demo_infrastructure_side_vehicle_components
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_demo_infrastructure_side_vehicle_components
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  math::math
  math::math_openmp
  geom::geom
  fltr::fltr
  ${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_demo_infrastructure_side_vehicle_components PROPERTIES OUTPUT_NAME demo_infrastructure_side_vehicle_components PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_demo_infrastructure_side_vehicle_components
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
