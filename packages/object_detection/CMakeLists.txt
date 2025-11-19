cmake_minimum_required(VERSION 2.8.3)
project(object_detection)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  src/object_detector_node.py
  src/simple_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)