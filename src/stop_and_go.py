#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import SegmentList, Twist2DStamped, FSMState, LEDPattern
from std_msgs.msg import ColorRGBA


class StopAndGo:
    def __init__(self):
        self.bot = "ferduckie"

        # States
        self.stopped = False
        self.stop_start_time = None
        self.cooldown_until = None

        # Parameters for red-line detection
        # x = forward direction in robot footprint frame
        # y = sideways direction
        self.min_x = 0.10
        self.max_x = 0.50
        self.max_abs_y = 0.25  # detection area
        self.required_segments = 3  # easier to trigger (red segments)
        self.stop_time = 5.0
        self.cooldown_time = 4.0

        # Publishers
        self.cmd_pub = rospy.Publisher(
            "/" + self.bot + "/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )
        self.fsm_pub = rospy.Publisher(
            "/" + self.bot + "/fsm_node/mode",
            FSMState,
            queue_size=1
        )
        self.led_pub = rospy.Publisher(
            "/" + self.bot + "/led_emitter_node/led_pattern",
            LEDPattern,
            queue_size=1
        )

        # Subscriber
        rospy.Subscriber(
            "/" + self.bot + "/ground_projection_node/lineseglist_out",
            SegmentList,
            self.segment_cb
        )

        # Timer for state handling
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("StopAndGo node started for %s", self.bot)

    def publish_fsm_mode(self, mode_name):
        msg = FSMState()
        msg.state = mode_name
        self.fsm_pub.publish(msg)

    def publish_stop_cmd(self):
        if rospy.is_shutdown():
            return

        msg = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0

        try:
            self.cmd_pub.publish(msg)
        except rospy.ROSException:
            pass

    def set_front_leds_red(self):
        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        off = ColorRGBA(0.0, 0.0, 0.0, 1.0)

        led_msg = LEDPattern()

        # LEDs
        led_msg.rgb_vals = [red, off, red, off, off]

        self.led_pub.publish(led_msg)

    def reset_leds(self):
        off = ColorRGBA(0.0, 0.0, 0.0, 1.0)

        led_msg = LEDPattern()
        led_msg.rgb_vals = [off, off, off, off, off]

        self.led_pub.publish(led_msg)

    def is_red_stop_line_segment(self, seg):
        if seg.color != 2:
            return False

        p1 = seg.points[0]
        p2 = seg.points[1]

        mid_x = (p1.x + p2.x) / 2.0
        mid_y = (p1.y + p2.y) / 2.0

        # Must be in front of the robot
        if mid_x < self.min_x or mid_x > self.max_x:
            return False

        # Must be close to the center
        if abs(mid_y) > self.max_abs_y:
            return False
     
        # Orientation filter removed: stop lines may appear at slight angles depending on robot heading, position filtering is enough

        return True

    def detect_red_line_ahead(self, msg):
        count = 0

        for seg in msg.segments:
            if self.is_red_stop_line_segment(seg):
                count += 1

        return count >= self.required_segments

    def start_stop_behavior(self):
        self.stopped = True
        self.stop_start_time = rospy.Time.now()

        rospy.loginfo("Red stop line detected. Stopping robot.")

        # Interrupt lane following
        self.publish_fsm_mode("NORMAL_JOYSTICK_CONTROL")

        # Send stop command
        self.publish_stop_cmd()

        # Turn front LEDs red
        self.set_front_leds_red()

    def resume_lane_following(self):
        rospy.loginfo("Five seconds passed. Resuming lane following.")

        # Reset LEDs
        self.reset_leds()

        # Resume normal lane following
        self.publish_fsm_mode("LANE_FOLLOWING")

        self.stopped = False
        self.stop_start_time = None
        self.cooldown_until = rospy.Time.now() + rospy.Duration(self.cooldown_time)

    def segment_cb(self, msg):
        now = rospy.Time.now()

        # Ignore detections while already stopped
        if self.stopped:
            return

        # Ignore detections during cooldown
        if self.cooldown_until is not None and now < self.cooldown_until:
            return

        red_count = sum(1 for s in msg.segments if s.color == 2)
        rospy.logdebug("Red segments seen: %d", red_count)

        if self.detect_red_line_ahead(msg):
            self.start_stop_behavior()

    def timer_cb(self, event):
        if rospy.is_shutdown():
            return

        if not self.stopped:
            return

        # Keep publishing stop command while paused
        self.publish_stop_cmd()

        elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()

        if elapsed >= self.stop_time:
            self.resume_lane_following()


if __name__ == "__main__":
    rospy.init_node("stop_and_go")
    node = StopAndGo()
    rospy.spin()
