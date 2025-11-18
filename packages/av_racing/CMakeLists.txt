cmake_minimum_required(VERSION 3.0.2)
project(av_racing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  src/racing_controller.py
  src/stopline_brake.py
  src/pure_pursuit_controller.py
  src/yolo_obstacle_detector.py
  src/racing_obstacle_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
