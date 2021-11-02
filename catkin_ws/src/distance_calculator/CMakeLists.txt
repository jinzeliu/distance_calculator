cmake_minimum_required(VERSION 2.8.3)
project(distance_calculator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  roslib
  grid_map_ros
  grid_map_core
  grid_map_msgs
)

find_package(PCL 1.2 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS
   grid_map_ros
   grid_map_core
   Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(/usr/local/lib)
include_directories(/usr/local/include/)

# message(STATUS "--------------------------------------------------------------{")
# get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
# message("inc_dirs = ${inc_dirs}")
# message("catkin = ${catkin_LIBRARIES}")
# message(STATUS "${INCLUDE_DIRECTORIES}")
# message(STATUS "--------------------------------------------------------------{")
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

####################################################################################
#### Calculate the trajectory distance.
####################################################################################
add_executable(dist_calc src/driver.cpp src/main.cpp
)
add_dependencies(dist_calc ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(dist_calc ${catkin_LIBRARIES}
)
