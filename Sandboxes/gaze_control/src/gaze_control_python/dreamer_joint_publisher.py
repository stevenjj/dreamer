#!/usr/bin/env python
import rospy
import time
import xml.dom.minidom
import math
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
        self.free_joints = {}  
        self.zeros = get_param("zeros")
        self.dependent_joints = get_param("dependent_joints", {})
        self.load_joint_information()

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

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

                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                # -- Not useful for the head --
                # For mimic joints
                mimic_tags = child.getElementsByTagName('mimic')
                if len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                # For each free joint, create with its zero starting position, min, max values, and if its continuous or not
                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                # Add and set position to 0
                joint['position'] = zeroval

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

        #self.print_joints()

    def print_joints(self):
        print "All Joints"
        for joint in self.joint_list:
            print joint
        print "Controllable Joints"
        for joint_name in self.joint_list:
            if joint_name not in self.dependent_joints:
                print joint_name

        #print self.free_joints

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        for i, name in enumerate(self.joint_list):
            joint = None

            # Add Free Joint
            if name in self.free_joints:
                joint = self.free_joints[name]
                factor = 1
                offset = 0
            # Add Dependent Joint
            elif name in self.dependent_joints:
                param = self.dependent_joints[name]
                parent = param['parent']
                joint = self.free_joints[parent]
                factor = param.get('factor', 1)
                offset = param.get('offset', 0)

            joint_position = joint['position'] * factor + offset
            #print name, ":", joint['position']

            msg.name.append(str(name))
            msg.position.append(joint_position)
            self.pub.publish(msg)


