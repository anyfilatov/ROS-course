#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from msg.msg import Speech

def talker():
    pub = rospy.Publisher('accountant', Speech, queue_size=10)
    rospy.init_node('lab2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    familyNames = ["Ivanov",  "Sidorov"]
    names = ["Ivan",  "Sidr"]
    while not rospy.is_shutdown():
        msg = Speech()
        msg.last_name = familyNames[random.randint(0,1)]
        msg.first_name = names[random.randint(0,1)]
        msg.salary = random.randint(90, 100)
        rospy.loginfo(msg.last_name+msg.first_name+str(msg.salary))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass