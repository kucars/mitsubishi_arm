cmake_minimum_required(VERSION 2.8.3)
project(mitsubishi_arm_hardware_interface)
catkin_python_setup() 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  hardware_interface
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
  ${catkin_INCLUDE_DIRS}
)


add_executable(mitsubishi_arm_hardware_interface src/MitsubishiArmInterface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mitsubishi_arm_hardware_interface
   ${catkin_LIBRARIES}
)

add_executable(com src/com.c)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/joint_state_publisher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

