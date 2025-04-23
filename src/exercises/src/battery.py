#!/usr/bin/env python

import rospy
from exercises.srv import SetLED
from rospy import Timer, Duration
from std_msgs.msg import Bool

class Battery:
    def __init__(self):
        rospy.init_node('battery_node')
        self.client = rospy.ServiceProxy('set_led', SetLED)
        rospy.wait_for_service('set_led')
        self.is_full = True
        rospy.loginfo("Battery node starting (initially full).")

        # Timer for toggling battery state
        self.toggle_timer = Timer(Duration(7 if self.is_full else 3),
                                  self.toggle_battery,
                                  oneshot=True)

        rospy.spin()

    def toggle_battery(self, event):
        self.is_full = not self.is_full
        state_str = "full" if self.is_full else "empty"
        rospy.loginfo(f"Battery is now {state_str}.")

        # Choose LED index 0 for this example
        led_idx = 0
        # Turn LED off if battery full, on if empty
        try:
            resp = self.client(led_idx, int(not self.is_full))
            if resp.success:
                rospy.loginfo(f"Successfully set LED {led_idx} to {int(not self.is_full)}")
            else:
                rospy.logwarn("Service call failed or invalid LED number.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call exception: {e}")

        # Schedule next toggle (7 s when full, 3 s when empty)
        next_interval = 7 if self.is_full else 3
        self.toggle_timer = Timer(Duration(next_interval),
                                  self.toggle_battery,
                                  oneshot=True)

if __name__ == '__main__':
    Battery()
