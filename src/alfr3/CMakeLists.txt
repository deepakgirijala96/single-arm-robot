cmake_minimum_required(VERSION 2.8.3)
project(alfr3)

## ----------------------------------------------------------------------------
## Find required catkin packages
## ----------------------------------------------------------------------------
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

find_package(roslaunch)

## ----------------------------------------------------------------------------
## Catkin package
## ----------------------------------------------------------------------------
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

## ----------------------------------------------------------------------------
## Install directories (URDF, meshes, config, launch)
## ----------------------------------------------------------------------------
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

