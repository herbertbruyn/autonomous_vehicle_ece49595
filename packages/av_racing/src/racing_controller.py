#!/usr/bin/env python3
"""
Racing controller node

Subscribes:
  ~lane_pose (duckietown_msgs/LanePose)  # usually /<NS>/lane_filter_node/pose

Publishes:
  /car_cmd_switch_node/cmd (duckietown_msgs/Twist2DStamped)

Strategy:
  - Steering: omega = k_phi * phi + k_d * d
  - Speed:    v = v_base * exp(-a_phi*|phi|) * exp(-a_d*|d|) * exp(-a_kappa*curvature)
  - If status != NORMAL or in_lane == False -> cap v conservatively
"""
import math
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped

NORMAL = 0

class RacingController:
    def __init__(self):
        # --- tunable parameters ---
        self.v_base = rospy.get_param('~v_base', 0.35)
        self.v_min  = rospy.get_param('~v_min',  0.10)
        self.v_max  = rospy.get_param('~v_max',  0.60)
        self.k_phi  = rospy.get_param('~k_phi',  3.5)
        self.k_d    = rospy.get_param('~k_d',    8.0)
        self.alpha_phi   = rospy.get_param('~alpha_phi',   2.0)
        self.alpha_d     = rospy.get_param('~alpha_d',     3.0)
        self.alpha_kappa = rospy.get_param('~alpha_kappa', 0.5)
        self.cmd_topic   = rospy.get_param('~cmd_topic','/car_cmd_switch_node/cmd')
        pose_topic       = rospy.get_param('~pose_topic','~lane_pose')

        # --- pubs/subs ---
        self.pub_cmd = rospy.Publisher(self._ns(self.cmd_topic),
                                       Twist2DStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber(pose_topic, LanePose,
                                         self.cb_pose, queue_size=1)

    def _ns(self, t: str) -> str:
        if t.startswith('/'):
            return t
        ns = rospy.get_namespace().rstrip('/')
        return f"{ns}/{t}" if ns else f"/{t}"

    def cb_pose(self, msg: LanePose):
        # Errors from lane pose
        phi = float(msg.phi)   # heading error (rad)
        d   = float(msg.d)     # lateral offset (m)

        # Steering: fast, simple baseline
        omega = self.k_phi * phi + self.k_d * d

        # Turn tightness proxy
        kappa = abs(float(msg.curvature))

        # Dynamic speed scheduling
        v = self.v_base
        v *= math.exp(-self.alpha_phi   * abs(phi))
        v *= math.exp(-self.alpha_d     * abs(d))
        v *= math.exp(-self.alpha_kappa * kappa)

        # Safety fallback on degraded localization
        if msg.status != NORMAL or (hasattr(msg, 'in_lane') and not msg.in_lane):
            v = min(v, 0.15)

        # Clamp and publish
        v = max(self.v_min, min(self.v_max, v))
        out = Twist2DStamped()
        out.header.stamp = rospy.Time.now()
        out.v     = v
        out.omega = omega
        self.pub_cmd.publish(out)

if __name__ == "__main__":
    rospy.init_node('racing_controller', anonymous=False)
    RacingController()
    rospy.spin()
