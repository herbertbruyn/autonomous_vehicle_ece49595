#!/usr/bin/env python3
"""
Racing Obstacle Avoidance Controller - AGGRESSIVE VERSION
Optimized for competitive racing with minimal speed loss

Subscribes:
  ~obstacle_detected (std_msgs/Bool)              # From obstacle detector
  ~obstacle_distance (std_msgs/Float32)           # From obstacle detector
  ~obstacle_status (std_msgs/String)              # From obstacle detector
  ~racing_cmd (duckietown_msgs/Twist2DStamped)    # From pure pursuit controller
  ~lane_pose (duckietown_msgs/LanePose)           # For smart direction selection

Publishes:
  /car_cmd_switch_node/cmd (duckietown_msgs/Twist2DStamped)  # Final command to robot
"""

import rospy
from std_msgs.msg import Bool, Float32, String
from duckietown_msgs.msg import Twist2DStamped, LanePose

class ObstacleAvoidanceController:
    def __init__(self):
        # Racing parameters - AGGRESSIVE
        self.emergency_distance = rospy.get_param('~emergency_distance', 0.12)  # Only stop if VERY close
        self.avoid_distance = rospy.get_param('~avoid_distance', 0.50)  # Start avoiding
        self.avoid_omega_gain = rospy.get_param('~avoid_omega_gain', 3.5)  # Aggressive steering
        self.speed_reduction = rospy.get_param('~speed_reduction', 0.80)  # Keep 80% speed
        self.max_omega = rospy.get_param('~max_omega', 6.0)  # Maximum turning rate
        
        # Direction strategy
        self.direction_mode = rospy.get_param('~direction_mode', 'lane_position')
        # Options: 'lane_position', 'alternate', 'right', 'left'
        
        # State
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_status = "CLEAR"
        self.lane_d = 0.0  # Lateral offset from lane center
        self.last_racing_cmd = None
        self.last_avoid_direction = 1  # 1 = right, -1 = left
        
        # Stats
        self.total_frames = 0
        self.avoidance_frames = 0
        
        # Publishers
        self.pub_cmd = rospy.Publisher('/car_cmd_switch_node/cmd',
                                      Twist2DStamped, queue_size=1)
        
        # Subscribers
        self.sub_obstacle = rospy.Subscriber('~obstacle_detected', Bool,
                                            self.callback_obstacle, queue_size=1)
        self.sub_distance = rospy.Subscriber('~obstacle_distance', Float32,
                                            self.callback_distance, queue_size=1)
        self.sub_status = rospy.Subscriber('~obstacle_status', String,
                                          self.callback_status, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber('~lane_pose', LanePose,
                                             self.callback_lane_pose, queue_size=1)
        self.sub_racing_cmd = rospy.Subscriber('~racing_cmd', Twist2DStamped,
                                              self.callback_racing_cmd, queue_size=1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("RACING Obstacle Avoidance Controller initialized")
        rospy.loginfo(f"  Emergency stop:   {self.emergency_distance:.2f}m (VERY close only!)")
        rospy.loginfo(f"  Avoid distance:   {self.avoid_distance:.2f}m")
        rospy.loginfo(f"  Omega gain:       {self.avoid_omega_gain:.2f} (aggressive)")
        rospy.loginfo(f"  Speed reduction:  {self.speed_reduction*100:.0f}% (minimal)")
        rospy.loginfo(f"  Max omega:        {self.max_omega:.2f}")
        rospy.loginfo(f"  Direction mode:   {self.direction_mode}")
        rospy.loginfo("=" * 60)
    
    def callback_obstacle(self, msg):
        """Update obstacle detection status"""
        self.obstacle_detected = msg.data
    
    def callback_distance(self, msg):
        """Update obstacle distance"""
        self.obstacle_distance = msg.data
    
    def callback_status(self, msg):
        """Update obstacle status string"""
        self.obstacle_status = msg.data
    
    def callback_lane_pose(self, msg):
        """Get current lane position for smart direction selection"""
        self.lane_d = msg.d  # Lateral offset: negative = left, positive = right
    
    def get_avoid_direction(self):
        """Determine which way to steer around obstacle"""
        
        if self.direction_mode == 'lane_position':
            # SMART: Steer away from current position
            # If we're on left (d < 0), steer right (+1)
            # If we're on right (d > 0), steer left (-1)
            # If centered, alternate
            if abs(self.lane_d) < 0.05:  # Centered (within 5cm)
                self.last_avoid_direction *= -1  # Alternate
                return self.last_avoid_direction
            else:
                direction = 1 if self.lane_d < 0 else -1
                self.last_avoid_direction = direction
                return direction
        
        elif self.direction_mode == 'alternate':
            # Alternate each time
            self.last_avoid_direction *= -1
            return self.last_avoid_direction
        
        elif self.direction_mode == 'right':
            return 1
        
        elif self.direction_mode == 'left':
            return -1
        
        else:
            return 1  # Default right
    
    def calculate_avoidance_omega(self, base_omega, distance):
        """Calculate steering adjustment - AGGRESSIVE for racing"""
        
        # Urgency increases as we get closer
        if distance < self.avoid_distance:
            urgency = 1.0 - (distance / self.avoid_distance)
            urgency = max(0.0, min(1.0, urgency))
        else:
            return base_omega  # No adjustment needed
        
        # Get direction
        direction = self.get_avoid_direction()
        
        # Calculate avoidance steering
        avoid_omega = direction * self.avoid_omega_gain * urgency
        
        # Add to base steering (don't replace it!)
        total_omega = base_omega + avoid_omega
        
        # Clamp to reasonable limits (don't spin out)
        total_omega = max(-self.max_omega, min(self.max_omega, total_omega))
        
        return total_omega
    
    def callback_racing_cmd(self, msg):
        """Main control logic - RACING OPTIMIZED"""
        self.total_frames += 1
        self.last_racing_cmd = msg
        
        # Create output command
        cmd_out = Twist2DStamped()
        cmd_out.header.stamp = rospy.Time.now()
        
        if self.obstacle_detected and self.obstacle_distance < self.avoid_distance:
            self.avoidance_frames += 1
            
            if self.obstacle_distance < self.emergency_distance:
                # EMERGENCY: Too close, must stop
                cmd_out.v = 0.0
                cmd_out.omega = msg.omega  # Keep lane following active
                action = "EMERGENCY"
                
            else:
                # RACING MODE: Steer around + minimal speed loss
                cmd_out.v = msg.v * self.speed_reduction  # Keep most speed!
                cmd_out.omega = self.calculate_avoidance_omega(msg.omega, 
                                                               self.obstacle_distance)
                
                direction_str = "RIGHT" if (cmd_out.omega > msg.omega) else "LEFT"
                action = f"AVOIDING {direction_str}"
                
                # Log avoidance maneuvers
                if self.avoidance_frames % 10 == 1:
                    rospy.loginfo(f"[{action}] Dist: {self.obstacle_distance:.2f}m | "
                                f"Lane d: {self.lane_d:.3f} | "
                                f"v: {msg.v:.2f}->{cmd_out.v:.2f} | "
                                f"omega: {msg.omega:.2f}->{cmd_out.omega:.2f}")
        
        else:
            # CLEAR: Full racing speed
            cmd_out.v = msg.v
            cmd_out.omega = msg.omega
        
        # Publish final command
        self.pub_cmd.publish(cmd_out)
        
        # Periodic statistics
        if self.total_frames % 300 == 0:  # Every ~3 seconds
            avoidance_pct = (self.avoidance_frames / self.total_frames * 100) if self.total_frames > 0 else 0
            rospy.loginfo(f"Stats: {avoidance_pct:.1f}% time avoiding obstacles")

if __name__ == "__main__":
    rospy.init_node('obstacle_avoidance_controller', anonymous=False)
    ObstacleAvoidanceController()
    rospy.spin()
