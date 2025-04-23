#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool

TOTAL = 0

def handle_reset(req):
    global TOTAL
    TOTAL = 0
    print("Total count reset to 0")
    return True, "Total count reset to 0"

def callback(data, counter_publisher):
    global TOTAL
    TOTAL += data.data
    msg = Int64()
    msg.data = TOTAL
    counter_publisher.publish(msg)
    print("Total count: ", msg.data)

def listener():
    rospy.init_node('number_counter', anonymous=True)
    counter_publisher = rospy.Publisher('/number_count', Int64, queue_size=10)
    rospy.Service('reset_number_count', SetBool, handle_reset)
    rospy.Subscriber("/number", Int64, callback,counter_publisher)
    rospy.spin()

if __name__ == '__main__':
    listener()