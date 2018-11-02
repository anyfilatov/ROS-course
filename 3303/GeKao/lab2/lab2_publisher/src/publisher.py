# !/usr/bin/env python

import rospy
from lab2_msg.msg import Msg
import random


class Publisher_Msg:
    def generate_string(self):
        generate_str = ""
        num = random.randint(0, 999) % 9 + 8
        for i in range(num):
            rand_str = str(random.randint(0, 999) % 26 + 65)
            generate_str += rand_str

        return generate_str

    def publisher(self):
        pub = rospy.Publisher('lab2_msg/Msg',Msg,queue_size=10)
        rospy.init_node('publisher',anonymous=True)
        rospy.loginfo("I give information!\n")
        rospy.sleep(1)
        rate = rospy.Rate(10)

        for i in range(10):
            msg = Msg()
            generate_str = self.generate_string()+str(random.randint(0,999)%2+49)
            msg.text = generate_str
            pub.publish(msg)
            rospy.loginfo('Msg:\"%s\"'%msg.text)
            rate.sleep()

if __name__ == '__main__':
    try:
        Publisher_Msg().publisher()
    except:
        rospy.loginfo("Publisher Msg")




