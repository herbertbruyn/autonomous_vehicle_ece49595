#!/bin/bash
source /environment.sh
dt-launchfile-init

# 1) ROS master
roscore &
sleep 2

# 2) Perception stack (dt-core).
#    Order: camera -> (line_detector) -> ground_projection -> lane_filter -> stop_line_filter
#           (lane_filter publishes LanePose; stop_line_filter publishes StopLineReading)
rosrun line_detector      line_detector_node.py &
rosrun ground_projection  ground_projection_node.py &
rosrun lane_filter        lane_filter_node.py &
rosrun stop_line_filter   stop_line_filter_node.py &

# Start ToF sensor driver (built-in for DB21J)
rosrun front_center_tof_driver front_center_tof_driver_node.py &
sleep 2

# 3) Our nodes:
# stopline_brake listens to "~stop_line_reading" from stop_line_filter and brakes when needed
rosrun av_racing stopline_brake.py &
sleep 0.5

# ToF obstacle detector (uses built-in Time-of-Flight sensor)
rosrun av_racing tof_obstacle_detector.py &
sleep 0.5

# Obstacle avoidance controller with proper topic remapping
rosrun av_racing obstacle_avoidance_controller.py \
  racing_cmd:=/pure_pursuit_racer/racing_cmd \
  lane_pose:=/lane_filter_node/lane_pose &
sleep 0.5

# main blocking process - pure pursuit controller (now publishes to avoidance controller)
dt-exec rosrun av_racing pure_pursuit_controller.py
#dt-exec rosrun av_racing racing_controller.py

dt-launchfile-join
