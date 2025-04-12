#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

class MyController:
    def __init__(self):
        rospy.init_node('my_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        self.rate = rospy.Rate(10)

    def move(self, distance, speed):
        self.vel_msg.linear.x = speed

        if speed < 0:
            print("Moving backward")
        else:
            print("Moving forward")

        while rospy.Time.now().to_sec() == 0:
            rospy.sleep(0.1)

        current_distance = 0.0
        t0 = rospy.Time.now().to_sec()
        
        while current_distance < distance and not rospy.is_shutdown():
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_distance = abs(speed) * (t1 - t0)
            print("Distance covered: ", current_distance,end="\r")
        
        self.stop()
            
    def turn(self, angle, omega):
        omega = omega * (np.pi / 180.0)
        self.vel_msg.angular.z = omega

        if omega < 0:
            print("Turning left")
        else:
            print("Turning right")

        while rospy.Time.now().to_sec() == 0:
            rospy.sleep(0.1)

        current_angle = 0.0
        t0 = rospy.Time.now().to_sec()

        while current_angle < angle and not rospy.is_shutdown():
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(omega) * (t1 - t0) * (180.0 / np.pi)
            print("Angle Rotated: ", current_angle,end="\r")
        
        self.stop()

    def circle(self, speed, omega):
        print("Moving in a circle")
        
        while not rospy.is_shutdown():
            self.vel_msg.linear.x = speed
            self.vel_msg.angular.z = omega
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        

    def execute(self):
        print("Moving in a circle")

        radius = 1.0
        speed = 0.1
        steps = 45

        angle_increment = 360.0 / steps
        distance = 2 * np.pi * radius / steps

        for i in range(steps):
            self.turn(angle_increment, 30.0)
            self.move(distance, speed)

    def stop(self):
        print("\nStopping")
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        controller = MyController()
        controller.execute()
        # controller.circle(0.5, 1.0) 


    except rospy.ROSInterruptException:
        pass
