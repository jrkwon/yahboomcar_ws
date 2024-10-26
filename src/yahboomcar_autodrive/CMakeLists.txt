cmake_minimum_required(VERSION 3.0.2)
project(yahboomcar_autodrive)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  tf
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(mylibs
  src/AutoDrive.cpp
)

set(LIBS
  mylibs
  ${catkin_LIBRARIES}
)


#add_executable(autodrive src/AutoDriveNode.cpp)
#target_link_libraries(autodrive ${LIBS})
