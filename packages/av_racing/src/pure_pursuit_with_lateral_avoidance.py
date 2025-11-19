#!/usr/bin/env python3
"""
Pure Pursuit Controller with Intelligent Obstacle Avoidance

This version includes:
- Lane following with pure pursuit
- Lateral obstacle avoidance (steers around obstacles while staying in lane)
- Speed modulation based on obstacle proximity
"""

import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped
from std_msgs.msg import Bool, String

NORMAL = 0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PurePursuitWithAvoidance:
    def __init__(self):
        # --- Original speed params ---
        self.v_base  = rospy.get_param('~v_base',  0.40)
        self.v_min   = rospy.get_param('~v_min',   0.10)
        self.v_max   = rospy.get_param('~v_max',   0.70)
        self.alpha_phi   = rospy.get_param('~alpha_phi',   2.0)
        self.alpha_d     = rospy.get_param('~alpha_d',     2.5)
        self.alpha_kappa = rospy.get_param('~alpha_kappa', 0.6)

        # --- Original pure pursuit params ---
        self.L0          = rospy.get_param('~lookahead_base', 0.20)  # m
        self.L_gain      = rospy.get_param('~lookahead_gain', 0.60)  # scales with speed
        self.kappa_limit = rospy.get_param('~kappa_limit',    3.0)   # 1/m
        self.baseline    = rospy.get_param('~baseline',       0.10)  # wheel separation

        # --- NEW: Obstacle avoidance params ---
        self.v_obstacle_stop = rospy.get_param('~v_obstacle_stop', 0.0)
        self.v_obstacle_slow = rospy.get_param('~v_obstacle_slow', 0.20)
        self.obstacle_mode = rospy.get_param('~obstacle_mode', 'avoid')  # 'stop', 'slow', or 'avoid'
        self.speed_transition_rate = rospy.get_param('~speed_transition_rate', 2.5)
        
        # Avoidance-specific parameters
        self.avoidance_offset = rospy.get_param('~avoidance_offset', 0.15)  # How far to shift laterally (m)
        self.avoidance_smoothness = rospy.get_param('~avoidance_smoothness', 0.3)  # Transition speed
        self.lane_width = rospy.get_param('~lane_width', 0.23)  # Duckietown lane width (m)
        self.max_lateral_offset = rospy.get_param('~max_lateral_offset', self.lane_width * 0.4) # Don't go beyond 40% of lane width
        
        # --- Topic names ---
        veh_name = rospy.get_param('~veh_name', 'mcqueen95')
        pose_topic = rospy.get_param('~pose_topic', f'/{veh_name}/lane_filter_node/lane_pose')
        cmd_topic = rospy.get_param('~cmd_topic', f'/{veh_name}/car_cmd_switch_node/cmd')
        obstacle_topic = rospy.get_param('~obstacle_topic', '/object_detector_node/obstacle_alert')
        classes_topic = rospy.get_param('~classes_topic', '/object_detector_node/detection_classes')

        # --- State tracking ---
        self.obstacle_detected = False
        self.detected_classes = ""
        self.last_v = self.v_min
        self.target_velocity = self.v_base
        self.current_velocity = 0.0
        self.last_update_time = rospy.Time.now()
        
        # Avoidance state
        self.target_lateral_offset = 0.0  # Target offset for obstacle avoidance
        self.current_lateral_offset = 0.0  # Current smoothed offset

        # --- Publishers and Subscribers ---
        self.pub_cmd = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber(pose_topic, LanePose, self.cb_pose, queue_size=1)
        self.sub_obstacle = rospy.Subscriber(obstacle_topic, Bool, 
                                            self.cb_obstacle, queue_size=1)
        self.sub_classes = rospy.Subscriber(classes_topic, String,
                                           self.cb_classes, queue_size=1)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Pure Pursuit with Lateral Obstacle Avoidance initialized")
        rospy.loginfo(f"Vehicle: {veh_name}")
        rospy.loginfo(f"Subscribing to: {pose_topic}")
        rospy.loginfo(f"Publishing to: {cmd_topic}")
        rospy.loginfo(f"Obstacle mode: {self.obstacle_mode}")
        rospy.loginfo(f"Base velocity: {self.v_base} m/s")
        rospy.loginfo(f"Lookahead: {self.L0} + {self.L_gain}*v")
        if self.obstacle_mode == 'avoid':
            rospy.loginfo(f"Avoidance offset: ±{self.avoidance_offset}m")
        rospy.loginfo("=" * 60)

    def cb_obstacle(self, msg: Bool):
        """Handle obstacle detection alerts"""
        was_detected = self.obstacle_detected
        self.obstacle_detected = msg.data
        
        if msg.data and not was_detected:
            rospy.logwarn(f"⚠️  Obstacle detected: {self.detected_classes if self.detected_classes else 'unknown'}")
        elif not msg.data and was_detected:
            rospy.loginfo("✓ Obstacle cleared")
    
    def cb_classes(self, msg: String):
        """Store detected object classes"""
        self.detected_classes = msg.data

    def smooth_velocity_transition(self, target_v, dt):
        """Smoothly transition velocity"""
        max_change = self.speed_transition_rate * dt
        velocity_diff = target_v - self.current_velocity
        
        if abs(velocity_diff) <= max_change:
            return target_v
        else:
            return self.current_velocity + math.copysign(max_change, velocity_diff)
    
    def smooth_lateral_offset(self, target_offset, dt):
        """Smoothly transition lateral offset for avoidance"""
        max_change = self.avoidance_smoothness * dt  # meters per second
        offset_diff = target_offset - self.current_lateral_offset
        
        if abs(offset_diff) <= max_change:
            return target_offset
        else:
            return self.current_lateral_offset + math.copysign(max_change, offset_diff)

    def speed_schedule(self, phi: float, d: float, k_ref: float) -> float:
        """Curvature- & error-aware speed profile"""
        v = self.v_base
        v *= math.exp(-self.alpha_phi   * abs(phi))
        v *= math.exp(-self.alpha_d     * abs(d))
        v *= math.exp(-self.alpha_kappa * abs(k_ref))
        return clamp(v, self.v_min, self.v_max)

    def cb_pose(self, msg: LanePose):
        """Process lane pose and compute control commands with avoidance"""
        rospy.loginfo_throttle(2.0, 
            f"[avoidance_pp] Received LanePose: d={msg.d:.3f}, phi={msg.phi:.3f}")
        
        # Calculate time delta
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time
        dt = max(0.001, min(dt, 0.1))
        
        # --- Extract lane-pose signals ---
        phi = float(msg.phi)         # heading error [rad]
        d   = float(msg.d)           # lateral offset [m]
        k_ref = float(msg.curvature) if hasattr(msg, 'curvature') else 0.0

        # --- Determine avoidance behavior ---
        if self.obstacle_detected:
            if self.obstacle_mode == 'avoid':
                # Steer to the side while slowing down
                # Determine which side to avoid (based on current position)
                # If we're already offset to the right, go more right; if left, go more left
                if d > 0.02:  # Already right of center
                    self.target_lateral_offset = self.avoidance_offset
                elif d < -0.02:  # Already left of center
                    self.target_lateral_offset = -self.avoidance_offset
                else:  # Centered - default to right
                    self.target_lateral_offset = self.avoidance_offset
                
                # Clamp to safe bounds
                self.target_lateral_offset = clamp(
                    self.target_lateral_offset,
                    -self.max_lateral_offset,
                    self.max_lateral_offset
                )
            else:
                # For 'stop' or 'slow' modes, don't add lateral offset
                self.target_lateral_offset = 0.0
        else:
            # No obstacle - return to center
            self.target_lateral_offset = 0.0
        
        # Smooth the lateral offset transition
        self.current_lateral_offset = self.smooth_lateral_offset(
            self.target_lateral_offset, dt
        )
        
        # --- Apply avoidance offset to target position ---
        # Modify the effective lateral error to steer toward avoidance position
        d_effective = d - self.current_lateral_offset
        
        # --- Adaptive lookahead ---
        L_d = self.L0 + self.L_gain * max(self.last_v, self.v_min)

        # --- Local target point on lane centerline with avoidance offset ---
        if abs(k_ref) < 1e-6:
            # Straight: aim ahead with lateral offset
            x_t, y_t = L_d, -d_effective
        else:
            # Curve: approximate with circular arc
            R = 1.0 / k_ref
            theta = L_d * k_ref
            x_t = R * math.sin(theta)
            y_t = R * (1.0 - math.cos(theta)) - d_effective

        # --- Heading to target ---
        alpha = math.atan2(y_t, x_t) + phi

        # --- Pure Pursuit curvature ---
        kappa_cmd = 2.0 * math.sin(alpha) / max(L_d, 1e-3)
        kappa_cmd = clamp(kappa_cmd, -self.kappa_limit, self.kappa_limit)

        # --- Speed scheduling ---
        v_cmd = self.speed_schedule(phi, d, k_ref)

        # --- Safety fallback ---
        if msg.status != NORMAL or (hasattr(msg, 'in_lane') and not msg.in_lane):
            v_cmd = min(v_cmd, 0.15)

        # --- Obstacle speed adjustment ---
        if self.obstacle_detected:
            if self.obstacle_mode == 'stop':
                v_cmd = min(v_cmd, self.v_obstacle_stop)
                rospy.logwarn_throttle(1.0, 
                    f"🛑 STOPPING for obstacle: {self.detected_classes}")
            elif self.obstacle_mode == 'slow':
                v_cmd = min(v_cmd, self.v_obstacle_slow)
                rospy.logwarn_throttle(1.0, 
                    f"⚠️  SLOWING for obstacle: {self.detected_classes}")
            else:  # 'avoid' mode
                # Reduce speed but not as much since we're steering around
                v_cmd = min(v_cmd, self.v_obstacle_slow + 0.1)
                rospy.logwarn_throttle(1.0, 
                    f"🔄 AVOIDING obstacle ({self.detected_classes}): "
                    f"offset={self.current_lateral_offset:.3f}m")

        # --- Clamp to limits ---
        v_cmd = clamp(v_cmd, self.v_min, self.v_max)

        # --- Smooth velocity transition ---
        v_smooth = self.smooth_velocity_transition(v_cmd, dt)
        self.current_velocity = v_smooth

        # --- Map curvature to yaw rate ---
        omega_cmd = v_smooth * kappa_cmd

        # --- Publish chassis command ---
        self.last_v = v_smooth
        cmd = Twist2DStamped()
        cmd.header.stamp = current_time
        cmd.v = -v_smooth  # NOTE: robot needs negative velocity (inverted commands)
        cmd.omega = -omega_cmd
        self.pub_cmd.publish(cmd)
        
        status = ''
        if self.obstacle_detected:
            status = f' [AVOID: offset={self.current_lateral_offset:+.3f}m]'
        
        rospy.loginfo_throttle(2.0, 
            f"[avoidance_pp] v={cmd.v:.3f} m/s, ω={cmd.omega:.3f} rad/s{status}")


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_avoidance', anonymous=False)
    rospy.loginfo("[pure_pursuit_avoidance] node starting")
    try:
        PurePursuitWithAvoidance()
        rospy.loginfo("[pure_pursuit_avoidance] initialization done, spinning")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
