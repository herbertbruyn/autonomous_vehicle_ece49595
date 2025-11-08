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
sleep 2

# 3) Our nodes:
# stopline_brake listens to "~stop_line_reading" from stop_line_filter and brakes when needed
rosrun av_racing stopline_brake.py &
sleep 0.5
# main blocking process
dt-exec rosrun av_racing pure_pursuit_controller.py
#dt-exec rosrun av_racing racing_controller.py

dt-launchfile-join
