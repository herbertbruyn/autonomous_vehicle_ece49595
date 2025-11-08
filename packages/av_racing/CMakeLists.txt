cmake_minimum_required(VERSION 3.0.2)
project(av_racing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/av_racing/racing_controller.py
  src/av_racing/stopline_brake.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
