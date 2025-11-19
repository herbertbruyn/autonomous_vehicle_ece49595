#!/usr/bin/env python3
"""
Collision Avoidance Node - Safety Layer

Monitors obstacle detection and overrides racing controller when needed.
"""

import rospy
from std_msgs.msg import Bool, String
from duckietown_msgs.msg import LanePose, Twist2DStamped


class CollisionAvoidanceNode:
    def __init__(self):
        self.safe_velocity = rospy.get_param('~safe_velocity', 0.0)
        self.reduced_velocity = rospy.get_param('~reduced_velocity', 0.15)
        self.obstacle_timeout = rospy.get_param('~obstacle_timeout', 0.5)
        self.avoidance_mode = rospy.get_param('~avoidance_mode', 'stop')
        self.control_priority = rospy.get_param('~control_priority', True)
        self.k_phi = rospy.get_param('~k_phi', 3.0)
        self.k_d = rospy.get_param('~k_d', 5.0)
        
        self.obstacle_detected = False
        self.last_obstacle_time = rospy.Time(0)
        self.lane_pose = None
        self.detected_classes = ""
        
        self.pub_cmd = rospy.Publisher('/car_cmd_switch_node/cmd', 
                                      Twist2DStamped, queue_size=1)
        self.sub_obstacle = rospy.Subscriber('/object_detector_node/obstacle_alert', 
                                            Bool, self.cb_obstacle, queue_size=1)
        self.sub_classes = rospy.Subscriber('/object_detector_node/detection_classes',
                                           String, self.cb_classes, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber('~lane_pose', LanePose,
                                             self.cb_lane_pose, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        rospy.loginfo("="*60)
        rospy.loginfo("Collision Avoidance Node initialized")
        rospy.loginfo(f"Mode: {self.avoidance_mode}")
        rospy.loginfo("="*60)
    
    def cb_obstacle(self, msg):
        current_time = rospy.Time.now()
        if msg.data:
            self.obstacle_detected = True
            self.last_obstacle_time = current_time
        else:
            # Keep obstacle status for timeout period after last detection
            if (current_time - self.last_obstacle_time).to_sec() > self.obstacle_timeout:
                self.obstacle_detected = False
    
    def cb_classes(self, msg):
        self.detected_classes = msg.data
    
    def cb_lane_pose(self, msg):
        self.lane_pose = msg
    
    def control_loop(self, event):
        if not self.control_priority or self.lane_pose is None:
            return
        
        if not self.obstacle_detected:
            return
        
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        
        if self.avoidance_mode == 'stop':
            cmd.v = self.safe_velocity
            cmd.omega = 0.0
            rospy.logwarn_throttle(1.0, f"🛑 STOPPING - {self.detected_classes}")
        else:
            cmd.v = self.reduced_velocity
            phi = float(self.lane_pose.phi)
            d = float(self.lane_pose.d)
            cmd.omega = self.k_phi * phi + self.k_d * d
            rospy.logwarn_throttle(1.0, f"⚠️  SLOWING - {self.detected_classes}")
        
        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    rospy.init_node('collision_avoidance_node', anonymous=False)
    node = CollisionAvoidanceNode()
    rospy.spin()
