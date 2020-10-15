cmake_minimum_required(VERSION 3.0.2)
project(active_3d_planning_kr_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  kr_tracker_msgs
  actionlib
  actionlib_msgs
  tf2
  tf2_geometry_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES active_3d_planning_kr_interface
 CATKIN_DEPENDS 
  trajectory_msgs
  roscpp
  actionlib
  actionlib_msgs
  kr_tracker_msgs
  tf2
  tf2_geometry_msgs
)

###########
## Build ##
###########


include_directories(include ${catkin_INCLUDE_DIRS})

add_library(active_3d_planning_kr_tracker_interface src/active_3d_planning_kr_tracker_interface)

add_executable(kr_tracker_interface src/kr_tracker_interface)

target_link_libraries(kr_tracker_interface active_3d_planning_kr_tracker_interface ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_active_3d_planning_kr_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
