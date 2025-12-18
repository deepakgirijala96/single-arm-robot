cmake_minimum_required(VERSION 3.0.2)
project(roboticarm8)  # Changed from roboticarm8.SLDASM

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  xacro
  message_generation
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    urdf
    xacro
)
find_package(roslaunch)

# Install directories (config, launch, meshes, urdf)
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
