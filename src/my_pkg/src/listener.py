#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    print("Received Odometry data:")
    print(data.twist.twist)
    
def listener():
    rospy.init_node('my_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()