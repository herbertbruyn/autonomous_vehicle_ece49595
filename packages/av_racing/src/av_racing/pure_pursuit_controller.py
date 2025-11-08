#!/usr/bin/env python3
"""Pure Pursuit-style racing controller for Duckietown using LanePose."""

import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped

NORMAL = 0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class PurePursuitRacer:
    def __init__(self):
        # --- speed params (tunable via ROS params) ---
        self.v_base  = rospy.get_param('~v_base',  0.40)
        self.v_min   = rospy.get_param('~v_min',   0.10)
        self.v_max   = rospy.get_param('~v_max',   0.70)
        self.alpha_phi   = rospy.get_param('~alpha_phi',   2.0)
        self.alpha_d     = rospy.get_param('~alpha_d',     2.5)
        self.alpha_kappa = rospy.get_param('~alpha_kappa', 0.6)

        # --- pure pursuit params ---
        self.L0          = rospy.get_param('~lookahead_base', 0.20)  # m
        self.L_gain      = rospy.get_param('~lookahead_gain', 0.60)  # scales with speed
        self.kappa_limit = rospy.get_param('~kappa_limit',    3.0)   # 1/m
        self.baseline    = rospy.get_param('~baseline',       0.10)  # wheel separation (not used directly here)

        self.cmd_topic   = rospy.get_param('~cmd_topic', '/car_cmd_switch_node/cmd')
        pose_topic       = rospy.get_param('~pose_topic', '~lane_pose')

        # Publisher (chassis commands) and subscriber (lane pose)
        self.pub_cmd = rospy.Publisher(self.ns(self.cmd_topic), Twist2DStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber(pose_topic, LanePose, self.cb_pose, queue_size=1)

        self.last_v = self.v_min

    def ns(self, t: str) -> str:
        """Return an absolute, namespaced topic path."""
        if t.startswith('/'):
            return t
        ns = rospy.get_namespace().rstrip('/')
        return f"{ns}/{t}" if ns else f"/{t}"

    def speed_schedule(self, phi: float, d: float, k_ref: float) -> float:
        """Curvature- & error-aware speed profile."""
        v = self.v_base
        v *= math.exp(-self.alpha_phi   * abs(phi))
        v *= math.exp(-self.alpha_d     * abs(d))
        v *= math.exp(-self.alpha_kappa * abs(k_ref))
        return clamp(v, self.v_min, self.v_max)

    def cb_pose(self, msg: LanePose):
        # Extract lane-pose signals (from lane_filter)
        phi = float(msg.phi)         # heading error [rad]
        d   = float(msg.d)           # lateral offset [m]
        k_ref = float(msg.curvature) # reference curvature [1/m]

        # Adaptive lookahead: longer when you're faster
        L_d = self.L0 + self.L_gain * max(self.last_v, self.v_min)

        # Local target point (lookahead) on lane centerline
        if abs(k_ref) < 1e-6:
            # Straight: aim L_d ahead, pull sideways back to centerline by -d
            x_t, y_t = L_d, -d
        else:
            # Curve: approximate centerline with a circular arc of radius R=1/k_ref
            R = 1.0 / k_ref
            theta = L_d * k_ref
            x_t = R * math.sin(theta)
            y_t = R * (1.0 - math.cos(theta)) - d

        # Heading to target in robot frame; add lane-heading error
        alpha = math.atan2(y_t, x_t) + phi

        # Pure Pursuit curvature law, clamped
        kappa_cmd = 2.0 * math.sin(alpha) / max(L_d, 1e-3)
        kappa_cmd = clamp(kappa_cmd, -self.kappa_limit, self.kappa_limit)

        # Speed scheduling (curvature & error aware)
        v_cmd = self.speed_schedule(phi, d, k_ref)

        # Safety fallback: if lane pose unreliable, crawl
        if msg.status != NORMAL or (hasattr(msg, 'in_lane') and not msg.in_lane):
            v_cmd = min(v_cmd, 0.15)

        # Map curvature to yaw rate
        omega_cmd = v_cmd * kappa_cmd

        # Publish chassis command
        self.last_v = v_cmd
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = v_cmd
        cmd.omega = omega_cmd
        self.pub_cmd.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_racer', anonymous=False)
    PurePursuitRacer()
    rospy.spin()