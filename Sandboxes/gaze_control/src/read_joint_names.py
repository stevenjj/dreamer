#!/usr/bin/env python
import rospy
import time
import xml.dom.minidom
from sensor_msgs.msg import JointState

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class Custom_Joint_Publisher():
    def __init__(self):
        self.joint_list = []  # for maintaining the original order of the joints        
        self.load_joint_information()
        self.rate = rospy.Rate(1) # 1hz
        self.ROS_start_time = rospy.Time.now().to_sec()

    def load_joint_information(self):
        # Load Robot Description for ROS parameter server
        description = get_param('robot_description')
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
        self.print_joints()

    def print_joints(self):
        for joint in self.joint_list:
            print joint


    def loop(self):
        while not rospy.is_shutdown():
            ROS_current_time = rospy.Time.now().to_sec() - self.ROS_start_time
            print "ROS time (sec): ", ROS_current_time
            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('head_joint_publisher')
        custom_joint_publisher = Custom_Joint_Publisher() 
        custom_joint_publisher.loop()
    except rospy.ROSInterruptException:
        pass