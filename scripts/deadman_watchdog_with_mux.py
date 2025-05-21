#!/usr/bin/env python

import os
import rospy
import subprocess
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from topic_tools.srv import MuxSelect
from actionlib_msgs.msg import GoalID

class DeadmanWatchdog:
    def __init__(self):
        rospy.init_node('deadman_watchdog')

        self.FNULL = open(os.devnull, 'w')

        # === Parameters ===
        self.deadman_button_index = rospy.get_param('~deadman_button_index', 4)
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/robot/robotnik_base_control/cmd_vel')
        self.nav_input_topic = rospy.get_param('~nav_input_topic', '/robot/cmd_vel_nav')
        self.fallback_topic = rospy.get_param('~fallback_topic', '/robot/cmd_vel_stop')
        self.mux_name = rospy.get_param('~mux_name', '/robot/cmd_vel_mux')
        self.mux_service = '/' + self.mux_name + '/select'
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 0.5)  # seconds

        self.deadman_pressed = False
        self.last_joy_time = rospy.Time.now()
        self.prev_deadman_pressed = None  # to detect edge
        self.rate = rospy.Rate(10)

        # === Start Mux ===
        self.start_mux()

        # === Publishers and Subscribers ===
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.stop_pub = rospy.Publisher(self.fallback_topic, Twist, queue_size=1, latch=True)
        self.stop_pub.publish(Twist())  # initialize with stop
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        rospy.Subscriber('robot/joy', Joy, self.joy_callback)

        rospy.wait_for_service(self.mux_service)
        self.select_mux = rospy.ServiceProxy(self.mux_service, MuxSelect)

        rospy.loginfo("Deadman watchdog started")
        self.run()

    def start_mux(self):
        rospy.loginfo("Starting topic_tools/mux...")
        cmd = [
            'rosrun', 'topic_tools', 'mux',
            self.cmd_vel_topic,
            self.nav_input_topic, self.fallback_topic,
            "mux:=" + self.mux_name
        ]
        self.mux_process = subprocess.Popen(cmd, stdout=self.FNULL, stderr=self.FNULL)
        rospy.loginfo("Mux subprocess started")
        rospy.on_shutdown(self.shutdown_mux)
        rospy.sleep(1.0)  # Allow some time for mux to start

    def shutdown_mux(self):
        if self.mux_process and self.mux_process.poll() is None:
            rospy.loginfo("Shutting down mux process...")
            self.mux_process.terminate()

    def joy_callback(self, msg):
        self.last_joy_time = rospy.Time.now()
        if self.deadman_button_index < len(msg.buttons):
            self.deadman_pressed = bool(msg.buttons[self.deadman_button_index])
        else:
            rospy.logwarn_throttle(5, "Deadman button index out of range!")

    def cancel_goals(self):
        rospy.loginfo_throttle(1, "Cancelling move_base goals")
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

    def run(self):
        timeout_duration = rospy.Duration(self.watchdog_timeout)

        while not rospy.is_shutdown():
            # Watchdog: treat deadman as released if timeout exceeded
            if rospy.Time.now() - self.last_joy_time > timeout_duration:
                self.deadman_pressed = False

            # Edge detection to avoid repeating actions unnecessarily
            if self.deadman_pressed != self.prev_deadman_pressed:
                if not self.deadman_pressed:
                    # Deadman released ? stop robot and cancel goals
                    try:
                        self.select_mux(self.fallback_topic)
                        self.cmd_pub.publish(Twist())
                        self.cancel_goals()
                    except rospy.ServiceException as e:
                        rospy.logerr_throttle(5, "Failed to switch mux: %s", str(e))
                else:
                    # Deadman pressed ? enable navigation
                    try:
                        self.select_mux(self.nav_input_topic)
                    except rospy.ServiceException as e:
                        rospy.logerr_throttle(5, "Failed to switch mux: %s", str(e))

            self.prev_deadman_pressed = self.deadman_pressed
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DeadmanWatchdog()
    except rospy.ROSInterruptException:
        pass
