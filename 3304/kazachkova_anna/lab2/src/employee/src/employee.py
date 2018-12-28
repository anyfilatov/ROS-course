#!/usr/bin/env python
import rospy
from msg.msg import Speech

salaries = []

def callback(speech):
    # rospy.loginfo(speech.first_name + speech.last_name + str(speech.salary))
    happy = True
    if(speech.first_name == "Ivan" and speech.last_name == "Sidorov"):
        for s in salaries:
            if s > speech.salary :
                happy = False
        if happy:
            rospy.loginfo("I am the happiest employee in the world!")
        else:
            rospy.loginfo("Today is not my day")
    else:
        salaries.append(speech.salary)

def listener():
    rospy.init_node('employee', anonymous=True)
    rospy.Subscriber("accountant", Speech, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()