#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
from utils import is_facing_goal, normalize_angle
import numpy as np

class MyController:
    def __init__(self,namespace = ""):
        self.namespace = namespace
        
        rospy.init_node('my_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(namespace + "/odom", Odometry, self.odom_callback)

        # meters
        self.x = 0.0
        self.y = 0.0
        
        # radians
        self.theta = 0.0

        self.rate = rospy.Rate(100)

        self.linear_kp=0.5
        self.angular_kp=1.0
        self.angle_tolerance=0.1
        self.dist_tolerance=0.01
        self.max_linear_speed=0.15
        self.max_angular_speed=0.3

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.theta = normalize_angle(yaw)

    def print_position(self):
        print("Current position: ", np.round(self.x,4), np.round(self.y,4))
        print("Current orientation: ", np.round(self.theta,4))

    def move_to_goal(self, goal_x, goal_y,reset_theta=True):
        # Compute the distance to the goal and the desired angle
        # Rotate to face the goal
        # Move towards the goal
        # Loop until the goal is reached
    
        
        print("\nMoving to goal: ", np.round(goal_x,4), np.round(goal_y,4))
        self.print_position()

        while not rospy.is_shutdown():
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.hypot(dx, dy)
            desired_angle = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_angle - self.theta)
            cmd = Twist()

            if abs(heading_error) > self.angle_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_kp * heading_error
                cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
            else:
                forward = is_facing_goal((self.x, self.y), (goal_x, goal_y), self.theta)
                direction = 1.0 if forward else -1.0
                cmd.linear.x = direction * self.linear_kp * distance
                cmd.linear.x = max(-self.max_linear_speed, min(cmd.linear.x, self.max_linear_speed))
                
                # Adding this made the robot move smoother, so I kept it 
                # Changing omega while moving forward

                cmd.angular.z = self.angular_kp * heading_error
                cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
            
            self.velocity_publisher.publish(cmd)
            if distance < self.dist_tolerance:
                print("Goal reached!")

                if reset_theta:
                    print("Resetting theta")
                    init_theta = 0.0 

                    cmd = Twist()
                    self.velocity_publisher.publish(cmd)
            
                    # increase the tolerance factor to make the robot more precise
                    # when rotating to the initial theta
                    tolerance_factor = 0.05
                    while abs(normalize_angle(init_theta - self.theta)) > self.angle_tolerance * tolerance_factor:
                        cmd.angular.z = self.angular_kp * normalize_angle(init_theta - self.theta)
                        cmd.angular.z = max(-self.max_angular_speed, min(cmd.angular.z, self.max_angular_speed))
                        self.velocity_publisher.publish(cmd)
                        self.rate.sleep()
                    print("Initial theta reached!")

                break
            self.rate.sleep()
        self.stop()
            

    def execute(self):
        # Wait for the odometry message to be received
        rospy.wait_for_message(self.namespace + "/odom", Odometry)
        self.move_to_goal(1.0,1.0,False)  
        self.move_to_goal(1.0,-1.0,False)
        self.move_to_goal(-1.0,-1.0,False)
        self.move_to_goal(-1.0,1.0,False)
        self.move_to_goal(0.0,0.0,True)


    def stop(self):
        print("Stopping")
        cmd = Twist()
        self.velocity_publisher.publish(cmd)
        self.print_position()


if __name__ == '__main__':
    try:
        controller = MyController()
        controller.execute()
    except rospy.ROSInterruptException:
        pass
