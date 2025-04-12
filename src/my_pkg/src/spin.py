#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def spin():
    rospy.init_node('my_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(10)  

    angle = 90.0  # Angle to rotate in degrees
    omega = 30.0    # Angular Velocity in degrees per second  

    omega = omega * (np.pi / 180.0)

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    current_angle = 0.0

    vel_msg.angular.z = omega

    # Added this because sometimes t0 was 0.0
    while rospy.Time.now().to_sec() == 0:
        rospy.sleep(0.1)

    t0 = rospy.Time.now().to_sec()

    while current_angle < angle and not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        t1 = rospy.Time.now().to_sec()
        current_angle = abs(omega) * (t1 - t0) * (180.0 / np.pi)
        print("Angle Rotated: ", current_angle,end="\r")
        
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        spin()
    except rospy.ROSInterruptException:
        pass
