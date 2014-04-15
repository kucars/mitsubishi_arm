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
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mitsubishi_arm_hardware_interface
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


add_library(mitsubishi_arm_hardware_interface src/MitsubishiArmInterface.cpp)
#add_library(mitsubishi_arm_hardware_interface_dummy src/MitsubishiArmInterfaceDummy.cpp)


add_executable(mitsubishi_arm_hardware_interface_node src/Robot.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mitsubishi_arm_hardware_interface_node mitsubishi_arm_hardware_interface
   ${catkin_LIBRARIES}
)

add_executable(joint_trajectory_action_client src/jointTrajectoryActionClient.cpp)
target_link_libraries(joint_trajectory_action_client mitsubishi_arm_hardware_interface
   ${catkin_LIBRARIES}
)
#add_executable(mitsubishi_arm_hardware_interface_dummy_node src/Robot.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(mitsubishi_arm_hardware_interface_dummy_node mitsubishi_arm_hardware_interface_dummy
#   ${catkin_LIBRARIES}
#)

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

