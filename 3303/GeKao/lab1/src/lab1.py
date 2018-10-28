# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi



class MoveAndBack():
    def __init__(self):

        # the name of node
        rospy.init_node("move_and_back",anonymous=False)
        # set the function when set off
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=1000)

        rate = rospy.Rate(1)
        for t in range(16):
            move_cmd = Twist()
            move_cmd.linear.x=1
            if t <5:
                move_cmd.angular.z=0.5
            elif t<8 and t>=5:
                move_cmd.angular.z=1
                move_cmd.linear.x=0.9
            elif t==8:
                move_cmd.angular.z=-10
            elif t>8 and t<12:
                move_cmd.angular.z=1
                move_cmd.linear.x=0.9
            else:
                move_cmd.angular.z=0.45
                move_cmd.linear.x=0.9
            rospy.loginfo("Move to position:\n 1) move_cmd.linear:x=%f y=%f z=%f\n 2) move_cmd.angular: x=%f y=%f z=%f"
                          %(move_cmd.linear.x,move_cmd.linear.y,move_cmd.linear.z,move_cmd.angular.x,move_cmd.angular.y,move_cmd.angular.z))
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

        


    def shutdown(self):
        # the turtle will stop,when the node is shutdown
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveAndBack()
    except:
        rospy.loginfo("Move-and-Back node terminated")




























