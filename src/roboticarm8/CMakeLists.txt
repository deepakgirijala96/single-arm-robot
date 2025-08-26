cmake_minimum_required(VERSION 3.0.2)
project(roboticarm8)  # Changed from roboticarm8.SLDASM

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  xacro
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

# Install directories (config, launch, meshes, urdf)
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
