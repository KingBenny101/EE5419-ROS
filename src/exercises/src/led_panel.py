#!/usr/bin/env python
import rospy
from exercises.srv import SetLED, SetLEDResponse

class LEDPanel:
    def __init__(self):
        rospy.init_node('led_panel')
        self.led_states = [0, 0, 0]

        self.srv = rospy.Service('set_led', SetLED, self.handle_set_led)

        rospy.Timer(rospy.Duration(1), self.print_states)

        rospy.loginfo("LED panel service ready.")
        rospy.spin()

    def handle_set_led(self, req):
        if 0 <= req.led_number < len(self.led_states):
            self.led_states[req.led_number] = req.state
            rospy.loginfo(f"LED {req.led_number} set to {req.state}")
            return SetLEDResponse(success=True)
        else:
            rospy.logwarn(f"Invalid LED number: {req.led_number}")
            return SetLEDResponse(success=False)

    def print_states(self, event):
            rospy.loginfo(f"LED states: {self.led_states}")

if __name__ == '__main__':
    LEDPanel()
