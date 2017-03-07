#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat

import tf

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class Detected_People_Manager():
    def __init__(self):
        self.list_of_people_markers = []
        self.person_of_interest = np.array([1,0,0])		

        self.subscriber = rospy.Subscriber('/human_boxes_3D', MarkerArray, self.detected_callback)

        self.person_of_interest = np.array([1,0,0])

        self.last_time_people_detected = 0 
        self.time_detection_threshold = 1 # Wait 1 second before 
        self.listener = tf.TransformListener()

    def detected_callback(self, markerArray):
         self.last_time_people_detected = rospy.Time.now().to_sec() 
         self.list_of_people_markers = markerArray.markers
         print 'people!', len(self.list_of_people_markers)

    #     m = self.list_of_people_markers[0]

    #     source_frame = "/camera_rgb_optical_frame"
    #     target_frame = "/my_world_neck"		 
    #     if self.listener.frameExists(source_frame) and self.listener.frameExists(target_frame):
    #         pose_in_source_frame = PoseStamped()
    #         pose_in_source_frame.header = m.header
    #         pose_in_source_frame.pose = m.pose
    #         pose_in_new_frame = self.listener.transformPose(target_frame, pose_in_source_frame)
    #         m.header.frame_id = target_frame
    #         m.marker.pose = pose_in_new_frame.pose


    #     (x,y,z) = m.pose.position.x, m.pose.position.y, m.pose.position.z

    #     print 'person location', (x,y,z)



    # def loop(self):
    #     current_time = rospy.Time.now().to_sec()
    #     if ((current_time - self.last_time_people_detected) > self.time_detection_threshold):
    #     #self.list_of_people_markers = []
    #         self.person_of_interest = np.array([1,0,0])

#        return