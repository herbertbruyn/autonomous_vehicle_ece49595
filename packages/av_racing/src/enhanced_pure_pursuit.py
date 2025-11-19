#!/usr/bin/env python3
"""
Enhanced Pure Pursuit Racing Controller with Obstacle Awareness

Based on pure_pursuit_controller.py but adds integrated obstacle detection support.

Subscribes:
  ~pose_topic (duckietown_msgs/LanePose)
  /object_detector_node/obstacle_alert (std_msgs/Bool)
  /object_detector_node/detection_classes (std_msgs/String)

Publishes:
  ~cmd_topic (duckietown_msgs/Twist2DStamped)
"""

import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped
from std_msgs.msg import Bool, String

NORMAL = 0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class EnhancedPurePursuitRacer:
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
        self.obstacle_mode = rospy.get_param('~obstacle_mode', 'slow')  # 'stop' or 'slow'
        self.speed_transition_rate = rospy.get_param('~speed_transition_rate', 2.5)  # m/s²
        
        # --- Topic names ---
        pose_topic_default = '/mcqueen95/lane_filter_node/lane_pose'
        cmd_topic_default  = '/mcqueen95/car_cmd_switch_node/cmd'
        
        pose_topic = rospy.get_param('~pose_topic', pose_topic_default)
        cmd_topic = rospy.get_param('~cmd_topic', cmd_topic_default)
        obstacle_topic = rospy.get_param('~obstacle_topic', '/object_detector_node/obstacle_alert')
        classes_topic = rospy.get_param('~classes_topic', '/object_detector_node/detection_classes')

        # --- State tracking ---
        self.obstacle_detected = False
        self.detected_classes = ""
        self.last_v = self.v_min
        self.target_velocity = self.v_base
        self.current_velocity = 0.0
        self.last_update_time = rospy.Time.now()

        # --- Publishers and Subscribers ---
        self.pub_cmd = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber(pose_topic, LanePose, self.cb_pose, queue_size=1)
        self.sub_obstacle = rospy.Subscriber(obstacle_topic, Bool, 
                                            self.cb_obstacle, queue_size=1)
        self.sub_classes = rospy.Subscriber(classes_topic, String,
                                           self.cb_classes, queue_size=1)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Enhanced Pure Pursuit Racer initialized")
        rospy.loginfo(f"Subscribing to: {pose_topic}")
        rospy.loginfo(f"Publishing to: {cmd_topic}")
        rospy.loginfo(f"Obstacle mode: {self.obstacle_mode}")
        rospy.loginfo(f"Base velocity: {self.v_base} m/s")
        rospy.loginfo(f"Lookahead: {self.L0} + {self.L_gain}*v")
        rospy.loginfo("=" * 60)

    def cb_obstacle(self, msg: Bool):
        """Handle obstacle detection alerts"""
        self.obstacle_detected = msg.data
        if msg.data:
            rospy.logwarn_throttle(1.0, 
                f"⚠️  Obstacle detected: {self.detected_classes if self.detected_classes else 'unknown'}")
    
    def cb_classes(self, msg: String):
        """Store detected object classes"""
        self.detected_classes = msg.data

    def smooth_velocity_transition(self, target_v, dt):
        """
        Smoothly transition from current velocity to target velocity
        to avoid jerky motion.
        
        Args:
            target_v: Target velocity (m/s)
            dt: Time since last update (seconds)
        
        Returns:
            New velocity value
        """
        max_change = self.speed_transition_rate * dt
        velocity_diff = target_v - self.current_velocity
        
        if abs(velocity_diff) <= max_change:
            return target_v
        else:
            return self.current_velocity + math.copysign(max_change, velocity_diff)

    def speed_schedule(self, phi: float, d: float, k_ref: float) -> float:
        """Curvature- & error-aware speed profile (from original)"""
        v = self.v_base
        v *= math.exp(-self.alpha_phi   * abs(phi))
        v *= math.exp(-self.alpha_d     * abs(d))
        v *= math.exp(-self.alpha_kappa * abs(k_ref))
        return clamp(v, self.v_min, self.v_max)

    def cb_pose(self, msg: LanePose):
        """Process lane pose and compute control commands"""
        rospy.loginfo_throttle(2.0, 
            f"[enhanced_pp] Received LanePose: d={msg.d:.3f}, phi={msg.phi:.3f}")
        
        # Calculate time delta for smooth transitions
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time
        
        # Clip dt to reasonable values
        dt = max(0.001, min(dt, 0.1))
        
        # --- Extract lane-pose signals ---
        phi = float(msg.phi)         # heading error [rad]
        d   = float(msg.d)           # lateral offset [m]
        k_ref = float(msg.curvature) # reference curvature [1/m]

        # --- Adaptive lookahead: longer when faster ---
        L_d = self.L0 + self.L_gain * max(self.last_v, self.v_min)

        # --- Local target point (lookahead) on lane centerline ---
        if abs(k_ref) < 1e-6:
            # Straight: aim L_d ahead, pull sideways back to centerline by -d
            x_t, y_t = L_d, -d
        else:
            # Curve: approximate centerline with a circular arc of radius R=1/k_ref
            R = 1.0 / k_ref
            theta = L_d * k_ref
            x_t = R * math.sin(theta)
            y_t = R * (1.0 - math.cos(theta)) - d

        # --- Heading to target in robot frame; add lane-heading error ---
        alpha = math.atan2(y_t, x_t) + phi

        # --- Pure Pursuit curvature law, clamped ---
        kappa_cmd = 2.0 * math.sin(alpha) / max(L_d, 1e-3)
        kappa_cmd = clamp(kappa_cmd, -self.kappa_limit, self.kappa_limit)

        # --- Speed scheduling (curvature & error aware) ---
        v_cmd = self.speed_schedule(phi, d, k_ref)

        # --- Safety fallback: if lane pose unreliable, crawl ---
        if msg.status != NORMAL or (hasattr(msg, 'in_lane') and not msg.in_lane):
            v_cmd = min(v_cmd, 0.15)

        # --- NEW: Obstacle avoidance logic ---
        if self.obstacle_detected:
            if self.obstacle_mode == 'stop':
                v_cmd = min(v_cmd, self.v_obstacle_stop)
                rospy.logwarn_throttle(1.0, 
                    f"🛑 STOPPING for obstacle: {self.detected_classes}")
            else:  # slow mode
                v_cmd = min(v_cmd, self.v_obstacle_slow)
                rospy.logwarn_throttle(1.0, 
                    f"⚠️  SLOWING for obstacle: {self.detected_classes}")

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
        cmd.v = v_smooth
        cmd.omega = omega_cmd
        self.pub_cmd.publish(cmd)
        
        rospy.loginfo_throttle(2.0, 
            f"[enhanced_pp] Command: v={cmd.v:.3f} m/s, ω={cmd.omega:.3f} rad/s"
            f"{' [OBSTACLE MODE]' if self.obstacle_detected else ''}")


if __name__ == '__main__':
    rospy.init_node('enhanced_pure_pursuit_racer', anonymous=False)
    rospy.loginfo("[enhanced_pp] node starting")
    EnhancedPurePursuitRacer()
    rospy.loginfo("[enhanced_pp] initialization done, spinning")
    rospy.spin()
