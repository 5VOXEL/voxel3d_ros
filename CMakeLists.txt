cmake_minimum_required(VERSION 2.8.3)
project(voxel3d_nodelet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  nodelet
  pcl_conversions
  pcl_ros
  std_msgs
)

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
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ./inc
)

link_directories(
  ${catkin_LIB_DIRS}
  ./lib
)

## Declare a C++ library
add_library(voxel3d_nodelet
  src/voxel3d_nodelet.cpp
)
target_link_libraries(voxel3d_nodelet voxel3d ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(voxel3d_nodelet voxel3d ${catkin_EXPORTED_LIBRARIES})
endif()
