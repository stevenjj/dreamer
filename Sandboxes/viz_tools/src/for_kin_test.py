#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState


#import sys
#sys.path.insert(0, './test_folder')
#from something import *

#import roslib
#roslib.load_manifest("gaze_control")

from gaze_control_python import modern_robotics as mr

#mr.Normalize()

#from gaze_control import Head_Kinematics
#from gaze_control import Head_Kinematics


class forwardKinTestNode():

    def __init__(self):
        #pub = rospy.Publisher('state', PointStamped, queue_size=10)
        markerPub = rospy.Publisher('eye_marker', Marker, queue_size=10)
        #rospy.Subscriber("joint_states", JointState, self.joint_states_callback)

        rospy.init_node('for_kin_test_node', anonymous=True)

        rate = rospy.Rate(100)

        self.robotMarker = Marker()
        self.robotMarker.header.frame_id = "lower_neck_pitch_link"
        self.robotMarker.header.stamp    = rospy.Time(0) #rospy.Time.now() #rospy.get_rostime()
        self.robotMarker.ns = "eye_marker"
        self.robotMarker.id = 0
        self.robotMarker.type = Marker.SPHERE #2 # sphere
        self.robotMarker.action = Marker.ADD
        self.robotMarker.pose.position.x = 1.
        self.robotMarker.pose.position.y = 1.
        self.robotMarker.pose.position.z = 1.
        self.robotMarker.pose.orientation.x = 0
        self.robotMarker.pose.orientation.y = 0
        self.robotMarker.pose.orientation.z = 0
        self.robotMarker.pose.orientation.w = 1.0
        self.robotMarker.scale.x = 1.0
        self.robotMarker.scale.y = 1.0
        self.robotMarker.scale.z = 1.0

        self.robotMarker.color.r = 0.0
        self.robotMarker.color.g = 1.0
        self.robotMarker.color.b = 0.0
        self.robotMarker.color.a = 1.0

        self.robotMarker.lifetime = rospy.Duration()

        while not rospy.is_shutdown():
            markerPub.publish(self.robotMarker)
            #print "sending marker", self.robotMarker
            rate.sleep()

    def joint_states_callback(self, joint_states):
        x = 1




forwardKinTestNode()