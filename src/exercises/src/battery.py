#!/usr/bin/env python
import rospy
from exercises.srv import SetLED
from rospy import Timer, Duration
from std_msgs.msg import Bool

class Battery:
    def __init__(self):
        rospy.init_node('battery_node')
        self.is_full = True
        rospy.loginfo("Battery node started (initially full).")

        self.on_time = 7
        self.charge_time = 3
        self.current_time = 0
        self.cycle_time = 0

        self.battery_clock()

    def battery_clock(self):
        while not rospy.is_shutdown():
            self.current_time += 1
            self.cycle_time += 1

            if self.is_full:
                if self.cycle_time >= self.on_time:
                    self.is_full = False
                    rospy.loginfo("Battery is now empty.")
                    self.cycle_time = 0
                    self.toggle_led(0, 1)
                    self.toggle_led(1, 1)
                    self.toggle_led(2, 1)
                    # self.toggle_led(3, 1)
            else:
                if self.cycle_time >= self.charge_time:
                    self.is_full = True
                    rospy.loginfo("Battery is now full.")
                    self.cycle_time = 0
                    self.toggle_led(0, 0)
                    self.toggle_led(1, 0)
                    self.toggle_led(2, 0)
        
            rospy.sleep(1)


    def toggle_led(self, led_idx = 0,led_on = 0 ):
        rospy.wait_for_service('set_led')
        try:
            client = rospy.ServiceProxy('set_led', SetLED)
            resp = client(led_idx, led_on)
            if resp.success:
                rospy.loginfo(f"Successfully set LED {led_idx} to {led_on}")
            else:
                rospy.logwarn("Service call failed or invalid LED number.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call exception: {e}")


if __name__ == '__main__':
    Battery()
