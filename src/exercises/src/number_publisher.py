#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

def publisher():
    rospy.init_node('number_publisher', anonymous=True)
    number_publisher = rospy.Publisher('/number', Int64, queue_size=10)

    msg = Int64() 
    rate = rospy.Rate(1)  
    msg.data = 1

    while not rospy.is_shutdown():
        number_publisher.publish(msg)
        print("Published number: ", msg.data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
