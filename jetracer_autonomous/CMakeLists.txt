cmake_minimum_required(VERSION 3.0.2)
project(jetracer_autonomous)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/autonomous_drive_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
