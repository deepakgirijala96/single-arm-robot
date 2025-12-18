cmake_minimum_required(VERSION 3.0.2)
project(lam_challenge_sim)

# Your original find_package command is untouched
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  xacro
  robot_state_publisher
  joint_state_publisher
  joint_state_publisher_gui
  controller_manager
  diff_drive_controller
  joint_state_controller
  gazebo_ros
  gazebo_ros_control
)

# Your original find_package for Gazebo is untouched
find_package(gazebo REQUIRED)

# Your original catkin_package command is untouched
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    urdf
    xacro
    robot_state_publisher
    joint_state_publisher
    joint_state_publisher_gui
    controller_manager
    diff_drive_controller
    joint_state_controller
    gazebo_ros
    gazebo_ros_control
)

# ================================================================
# == THE FINAL BUILD CODE WITH THE HARD-CODED PATH ==============
# ================================================================

# Tell the compiler where to look for header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  # == THIS IS THE FINAL FIX: Explicitly add the Gazebo 11 path ==
  /usr/include/gazebo-11
)

# This command compiles our .cpp file into a plugin library
add_library(visual_controller_plugin
  src/VisualControllerPlugin.cpp
)

# This command links our plugin against the necessary ROS and Gazebo libraries
target_link_libraries(visual_controller_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
