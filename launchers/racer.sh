#!/bin/bash
set -e

source /environment.sh
dt-launchfile-init

# 2) Load parameters from YAMLs so DTParam / rospy.get_param find them

# Lane filter params
rosparam load \
  "$(rospack find lane_filter)/config/lane_filter_node/default.yaml" \
  /lane_filter_node

# Line detector params
rosparam load \
  "$(rospack find line_detector)/config/line_detector_node/default.yaml" \
  /line_detector_node

# Stop line filter params
rosparam load \
  "$(rospack find stop_line_filter)/config/stop_line_filter_node/default.yaml" \
  /stop_line_filter

# Pure pursuit parameters
rosparam load \
  "$(rospack find av_racing)/config/av_racing.yaml" \
  /av_racing


# 3) Perception stack (dt-core)
rosrun line_detector      line_detector_node.py      &
rosrun ground_projection  ground_projection_node.py  &
rosrun lane_filter        lane_filter_node.py        &
rosrun stop_line_filter   stop_line_filter_node.py   &
sleep 2

# 4) Your nodes
rosrun av_racing stopline_brake.py &
sleep 0.5

# main blocking process
dt-exec roslaunch av_racing racing_with_avoidance.launch veh:=mcqueen95 avoidance_mode:=avoid

dt-launchfile-join
