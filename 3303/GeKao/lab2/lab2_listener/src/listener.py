#!/usr/bin/env python
import rospy
from lab2_msg.msg import Msg
import sys

class Listener():
    def __init__(self):
        self.cm = 0
        self.count_msgs = 10

    def callback(self, msg):
        rospy.loginfo(str(rospy.get_caller_id) + "I heard %s" % (msg.text))
        self.cm += 1
        log = ""
        if msg.text.isdigit():
            num = int(msg.text)
            if num % 2 == 0:
                rospy.loginfo("Msg:\"%s\" is true" % (msg.text))
            else:
                rospy.loginfo("Msg:\"%s\" is false" % (msg.text))
            if self.cm == self.count_msgs:
                rospy.spin()
        else:
            log = "The msg is not the number"
        return log

    def listen_msg(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('lab2_msg/Msg', Msg, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        Listener().listen_msg()
    except:
        rospy.loginfo("Listener Msg!")
