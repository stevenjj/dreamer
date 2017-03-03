#!/usr/bin/env python
import rospy
import time
import xml.dom.minidom
import math
from sensor_msgs.msg import JointState

head_joint_names = ['upper_neck_pitch']

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
        self.rate = rospy.Rate(60) # 1hz
        self.ROS_start_time = rospy.Time.now().to_sec()
        self.ROS_current_time = rospy.Time.now().to_sec()

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

        self.print_joints()

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


    def update_joint_position(self):
        #J3_name = 'upper_neck_pitch'
        J3_name = 'upper_neck_yaw'
        J3_max_val = self.free_joints[J3_name]['max']
        J3_min_val = self.free_joints[J3_name]['min']

        def bound(num):
            if num >= J3_max_val:
                return J3_max_val
            elif (num <= J3_min_val):
                return J3_min_val
            else:
                return num

        T = 6.
        frequency = 1.0/T
        angular_frequency = 2*3.14*frequency

        amp = abs( min([J3_max_val, J3_min_val]) ) /8.0
        bias = 0#(J3_max_val - J3_min_val)/2.0

        command = bound(amp*math.sin(angular_frequency*self.ROS_current_time) + bias)

        self.free_joints[J3_name]['position'] = command
        print J3_name, ":",  command


    def loop(self):
        while not rospy.is_shutdown():
            self.ROS_current_time = rospy.Time.now().to_sec() - self.ROS_start_time
            print "ROS time (sec): ", self.ROS_current_time

            self.update_joint_position()
            self.publish_joints()

            self.rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('head_joint_publisher')
        custom_joint_publisher = Custom_Joint_Publisher() 
        custom_joint_publisher.loop()
    except rospy.ROSInterruptException:
        pass