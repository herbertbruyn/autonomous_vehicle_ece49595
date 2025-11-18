#!/usr/bin/env python3
"""
ToF-based Obstacle Detection for DB21J
Uses the built-in Time-of-Flight sensor for fast, reliable obstacle detection

Subscribes:
  /front_center_tof_driver_node/range (sensor_msgs/Range)  # ToF sensor data

Publishes:
  ~obstacle_detected (std_msgs/Bool)       # True if obstacle in path
  ~obstacle_distance (std_msgs/Float32)    # Distance to obstacle in meters
  ~obstacle_status (std_msgs/String)       # Human-readable status
"""

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String

class ToFObstacleDetector:
    def __init__(self):
        # Parameters
        self.stop_distance = rospy.get_param('~stop_distance', 0.25)  # meters - emergency stop
        self.slow_distance = rospy.get_param('~slow_distance', 0.45)  # meters - start slowing
        self.warn_distance = rospy.get_param('~warn_distance', 0.70)  # meters - warning zone
        
        # State
        self.last_valid_distance = float('inf')
        self.reading_count = 0
        
        # Publishers
        self.pub_obstacle = rospy.Publisher('~obstacle_detected', Bool, queue_size=1)
        self.pub_distance = rospy.Publisher('~obstacle_distance', Float32, queue_size=1)
        self.pub_status = rospy.Publisher('~obstacle_status', String, queue_size=1)
        
        # Subscriber to ToF sensor (standard topic for DB21J)
        self.sub_range = rospy.Subscriber('/front_center_tof_driver_node/range',
                                         Range, self.callback_range, queue_size=1)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("ToF Obstacle Detector for DB21J initialized")
        rospy.loginfo(f"  Stop distance:  {self.stop_distance:.2f}m")
        rospy.loginfo(f"  Slow distance:  {self.slow_distance:.2f}m")
        rospy.loginfo(f"  Warn distance:  {self.warn_distance:.2f}m")
        rospy.loginfo("=" * 50)
    
    def callback_range(self, msg):
        """Process ToF sensor reading"""
        distance = msg.range
        
        # Check if valid reading (within sensor range)
        if distance < msg.min_range or distance > msg.max_range:
            # Invalid reading - use last known distance or assume clear
            distance = self.last_valid_distance
            status = "INVALID_READING"
            obstacle_detected = False
        else:
            # Valid reading
            self.last_valid_distance = distance
            
            # Determine status and obstacle detection
            if distance < self.stop_distance:
                status = "CRITICAL"
                obstacle_detected = True
            elif distance < self.slow_distance:
                status = "OBSTACLE_CLOSE"
                obstacle_detected = True
            elif distance < self.warn_distance:
                status = "OBSTACLE_WARNING"
                obstacle_detected = True
            else:
                status = "CLEAR"
                obstacle_detected = False
        
        # Publish results
        self.pub_obstacle.publish(Bool(data=obstacle_detected))
        self.pub_distance.publish(Float32(data=distance))
        self.pub_status.publish(String(data=status))
        
        # Logging (throttled)
        self.reading_count += 1
        if self.reading_count % 50 == 0:  # Log every 50 readings (~0.5 sec at 100Hz)
            if obstacle_detected:
                rospy.loginfo(f"ToF: {status} - Obstacle at {distance:.3f}m")
            else:
                rospy.logdebug(f"ToF: {status} - Distance {distance:.3f}m")

if __name__ == "__main__":
    rospy.init_node('tof_obstacle_detector', anonymous=False)
    ToFObstacleDetector()
    rospy.spin()

