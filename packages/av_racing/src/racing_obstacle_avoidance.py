#!/usr/bin/env python3
"""
Racing Obstacle Avoidance Controller with YOLO Detection
Optimized for competitive racing with minimal speed loss

Subscribes:
  ~obstacle_detected (std_msgs/Bool)              # From YOLO detector
  ~obstacle_position (geometry_msgs/Point)        # From YOLO detector (x=distance, y=lateral)
  ~racing_cmd (duckietown_msgs/Twist2DStamped)    # From pure pursuit controller
  ~lane_pose (duckietown_msgs/LanePose)           # For smart direction selection

Publishes:
  /car_cmd_switch_node/cmd (duckietown_msgs/Twist2DStamped)  # Final command to robot
"""

import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from duckietown_msgs.msg import Twist2DStamped, LanePose

class RacingObstacleAvoidance:
    def __init__(self):
        # Racing parameters - AGGRESSIVE
        self.emergency_distance = rospy.get_param('~emergency_distance', 0.15)  # Stop if very close
        self.avoid_distance = rospy.get_param('~avoid_distance', 0.60)  # Start avoiding
        self.avoid_omega_gain = rospy.get_param('~avoid_omega_gain', 4.0)  # Aggressive steering
        self.speed_reduction_close = rospy.get_param('~speed_reduction_close', 0.70)  # 70% speed when close
        self.speed_reduction_far = rospy.get_param('~speed_reduction_far', 0.90)  # 90% speed when far
        self.max_omega = rospy.get_param('~max_omega', 6.5)  # Maximum turning rate
        
        # Lateral offset influence
        self.lateral_gain = rospy.get_param('~lateral_gain', 2.0)  # How much lateral position affects steering
        
        # Direction strategy
        self.direction_mode = rospy.get_param('~direction_mode', 'smart')
        # Options: 'smart' (use obstacle lateral position), 'lane_position', 'alternate', 'right', 'left'
        
        # State
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_lateral = 0.0  # Negative = left, Positive = right
        self.lane_d = 0.0  # Lateral offset from lane center
        self.lane_phi = 0.0  # Heading error
        self.last_racing_cmd = None
        self.last_avoid_direction = 1  # 1 = right, -1 = left
        
        # Stats
        self.total_frames = 0
        self.avoidance_frames = 0
        self.emergency_stops = 0
        
        # Publishers
        self.pub_cmd = rospy.Publisher('/car_cmd_switch_node/cmd',
                                      Twist2DStamped, queue_size=1)
        
        # Subscribers
        self.sub_obstacle = rospy.Subscriber('~obstacle_detected', Bool,
                                            self.callback_obstacle, queue_size=1)
        self.sub_position = rospy.Subscriber('~obstacle_position', Point,
                                            self.callback_position, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber('~lane_pose', LanePose,
                                             self.callback_lane_pose, queue_size=1)
        self.sub_racing_cmd = rospy.Subscriber('~racing_cmd', Twist2DStamped,
                                              self.callback_racing_cmd, queue_size=1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("RACING Obstacle Avoidance Controller (YOLO) initialized")
        rospy.loginfo(f"  Emergency stop:   {self.emergency_distance:.2f}m")
        rospy.loginfo(f"  Avoid distance:   {self.avoid_distance:.2f}m")
        rospy.loginfo(f"  Omega gain:       {self.avoid_omega_gain:.2f}")
        rospy.loginfo(f"  Speed reduction:  {self.speed_reduction_close*100:.0f}% (close) / "
                     f"{self.speed_reduction_far*100:.0f}% (far)")
        rospy.loginfo(f"  Max omega:        {self.max_omega:.2f}")
        rospy.loginfo(f"  Direction mode:   {self.direction_mode}")
        rospy.loginfo("=" * 60)
    
    def callback_obstacle(self, msg):
        """Update obstacle detection status"""
        self.obstacle_detected = msg.data
    
    def callback_position(self, msg):
        """Update obstacle position (x=distance, y=lateral offset)"""
        self.obstacle_distance = msg.x
        self.obstacle_lateral = msg.y
    
    def callback_lane_pose(self, msg):
        """Get current lane position for smart direction selection"""
        self.lane_d = msg.d  # Lateral offset: negative = left, positive = right
        self.lane_phi = msg.phi  # Heading error
    
    def get_avoid_direction(self):
        """Determine which way to steer around obstacle"""
        
        if self.direction_mode == 'smart':
            # SMART: Steer away from obstacle's lateral position
            # If obstacle is on left (negative), steer right (+1)
            # If obstacle is on right (positive), steer left (-1)
            if abs(self.obstacle_lateral) < 0.05:  # Centered obstacle
                # Use lane position as tiebreaker
                if abs(self.lane_d) < 0.05:
                    self.last_avoid_direction *= -1  # Alternate
                    return self.last_avoid_direction
                else:
                    direction = 1 if self.lane_d < 0 else -1
            else:
                direction = -1 if self.obstacle_lateral > 0 else 1
            
            self.last_avoid_direction = direction
            return direction
        
        elif self.direction_mode == 'lane_position':
            # Steer away from current lane position
            if abs(self.lane_d) < 0.05:  # Centered
                self.last_avoid_direction *= -1
                return self.last_avoid_direction
            else:
                direction = 1 if self.lane_d < 0 else -1
                self.last_avoid_direction = direction
                return direction
        
        elif self.direction_mode == 'alternate':
            self.last_avoid_direction *= -1
            return self.last_avoid_direction
        
        elif self.direction_mode == 'right':
            return 1
        
        elif self.direction_mode == 'left':
            return -1
        
        else:
            return 1  # Default right
    
    def calculate_avoidance_omega(self, base_omega, distance, lateral_offset):
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
        # Base avoidance + lateral offset compensation
        avoid_omega = direction * self.avoid_omega_gain * urgency
        
        # Add lateral offset influence (steer harder if obstacle is more to the side)
        lateral_influence = -self.lateral_gain * lateral_offset * urgency
        avoid_omega += lateral_influence
        
        # Add to base steering (don't replace it!)
        total_omega = base_omega + avoid_omega
        
        # Clamp to reasonable limits
        total_omega = max(-self.max_omega, min(self.max_omega, total_omega))
        
        return total_omega
    
    def calculate_speed_reduction(self, distance):
        """Calculate speed reduction based on obstacle distance"""
        if distance >= self.avoid_distance:
            return 1.0  # No reduction
        
        # Linear interpolation between close and far reduction
        ratio = distance / self.avoid_distance
        reduction = self.speed_reduction_close + (self.speed_reduction_far - self.speed_reduction_close) * ratio
        return max(0.0, min(1.0, reduction))
    
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
                self.emergency_stops += 1
                action = "EMERGENCY STOP"
                
                if self.emergency_stops % 10 == 1:
                    rospy.logwarn(f"[{action}] Obstacle at {self.obstacle_distance:.2f}m!")
                
            else:
                # RACING MODE: Steer around + dynamic speed reduction
                speed_factor = self.calculate_speed_reduction(self.obstacle_distance)
                cmd_out.v = msg.v * speed_factor
                cmd_out.omega = self.calculate_avoidance_omega(msg.omega, 
                                                               self.obstacle_distance,
                                                               self.obstacle_lateral)
                
                direction_str = "RIGHT" if (cmd_out.omega > msg.omega) else "LEFT"
                action = f"AVOIDING {direction_str}"
                
                # Log avoidance maneuvers
                if self.avoidance_frames % 10 == 1:
                    rospy.loginfo(f"[{action}] Dist: {self.obstacle_distance:.2f}m | "
                                f"Lateral: {self.obstacle_lateral:.2f}m | "
                                f"Lane d: {self.lane_d:.3f} | "
                                f"v: {msg.v:.2f}->{cmd_out.v:.2f} ({speed_factor*100:.0f}%) | "
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
            rospy.loginfo(f"Stats: {avoidance_pct:.1f}% time avoiding | {self.emergency_stops} emergency stops")

if __name__ == "__main__":
    rospy.init_node('racing_obstacle_avoidance', anonymous=False)
    RacingObstacleAvoidance()
    rospy.spin()

