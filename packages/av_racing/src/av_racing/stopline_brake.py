#!/usr/bin/env python3
"""
Listen to StopLineReading and brake at the finish line.
"""
import rospy, time
from duckietown_msgs.msg import StopLineReading, Twist2DStamped

class StoplineBrake:
    def __init__(self):
        self.cmd_topic = rospy.get_param('~cmd_topic', '/car_cmd_switch_node/cmd')
        self.min_hold  = rospy.get_param('~hold_seconds', 1.0)  # brake hold time
        self.at_count_req = rospy.get_param('~confirm_frames', 3)

        self._at_counter = 0
        self._braked_until = 0.0

        self.pub_cmd = rospy.Publisher(self._ns(self.cmd_topic),
                                       Twist2DStamped, queue_size=1)
        rospy.Subscriber('~stop_line_reading', StopLineReading, self.cb_reading, queue_size=1)

    def _ns(self, t):
        if t.startswith('/'): return t
        ns = rospy.get_namespace().rstrip('/')
        return f"{ns}/{t}" if ns else f"/{t}"

    def cb_reading(self, msg: StopLineReading):
        now = time.time()
        # If already braking, keep holding until time elapses
        if now < self._braked_until:
            self._publish_brake()
            return

        # Debounce: require N consecutive "at stopline" frames
        if msg.at_stop_line:
            self._at_counter += 1
        else:
            self._at_counter = 0

        if self._at_counter >= self.at_count_req:
            self._braked_until = now + self.min_hold
            self._publish_brake()

    def _publish_brake(self):
        cmd = Twist2DStamped()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.pub_cmd.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('stopline_brake', anonymous=False)
    StoplineBrake()
    rospy.spin()
