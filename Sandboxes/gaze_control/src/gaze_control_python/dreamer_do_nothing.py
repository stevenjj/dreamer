#!/usr/bin/env python
import rospy
import dreamer_joint_publisher

class Dreamer_Head():
    def __init__(self):
        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()
        self.rate = rospy.Rate(1000) 

    def loop(self):
        while not rospy.is_shutdown():
            self.joint_publisher.publish_joints()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()