cmake_minimum_required(VERSION 3.0.2)
project(rover_arm_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs
  moveit_commander
  moveit_msgs
  move_base_msgs
  tf
)

## ========================================
##               BUILD STEPS
## ========================================

## Declare a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_arm_pkg
#  CATKIN_DEPENDS rospy std_msgs ...
#  DEPENDS system_lib
)

## ========================================
##               INSTALLATION
## ========================================

## 1. Install Python Scripts
## This ensures your .py files are executable and found by 'rosrun'
catkin_install_python(PROGRAMS
  scripts/arm_bridge.py
  scripts/joy_arm_commander.py
  scripts/autonomous_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 2. Install Launch Files
## Allows 'roslaunch rover_arm_pkg rover_arm.launch'
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 3. Install Config/URDF Files
## Ensures your robot description is available
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
