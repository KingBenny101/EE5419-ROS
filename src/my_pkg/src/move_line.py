#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_line():
    rospy.init_node('my_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    distance = 1.0  
    speed = 0.05    
    rate = rospy.Rate(10)  

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    current_distance = 0.0

    vel_msg.linear.x = speed

    # Added this because sometimes t0 was 0.0
    while rospy.Time.now().to_sec() == 0:
        rospy.sleep(0.1)

    t0 = rospy.Time.now().to_sec()

    while current_distance < distance and not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        t1 = rospy.Time.now().to_sec()
        current_distance = abs(speed) * (t1 - t0)
        print("Distance covered: ", current_distance,end="\r")
        
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        move_line()
    except rospy.ROSInterruptException:
        pass
