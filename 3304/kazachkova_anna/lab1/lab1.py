#!/usr/bin/env python
#license removed
import rospy
import random
from geometry_msgs.msg import Twist

def lab1():
	publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rospy.init_node('lab1', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		vel_msg = Twist()
		vel_msg.linear.x = random.randint(0,3)
		vel_msg.linear.y= random.randint(0,3)
		vel_msg.linear.z=random.randint(0,3)
		vel_msg.angular.x = random.randint(0,3)
		vel_msg.angular.y = random.randint(0,3)
		vel_msg.angular.z = random.randint(0,2)
		rospy.loginfo(vel_msg)
		publisher.publish(vel_msg)
		rate.sleep()
if __name__ == '__main__':
	try:
		lab1()
	except rospy.ROSInterruptException:
	        pass
