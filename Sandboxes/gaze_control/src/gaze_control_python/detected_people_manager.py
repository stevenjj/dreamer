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

        self.last_time_people_detected = 0 
        self.time_detection_threshold = 1 # Wait 1 second before 
        self.listener = tf.TransformListener()

    def detected_callback(self, markerArray):
        self.last_time_people_detected = rospy.Time.now().to_sec() 
        temporary_list_of_people_markers = markerArray.markers
        num_detected_people = len(temporary_list_of_people_markers)
        print 'Number of Detected People:', num_detected_people

        if not(num_detected_people > 0):
            return

        source_frame = "/camera_rgb_optical_frame"
        target_frame = "/my_world_neck"		 
        if self.listener.frameExists(source_frame) and self.listener.frameExists(target_frame):
            try:
                # Get T_SE3 Transform
                tf_latest_time = self.listener.getLatestCommonTime(source_frame, target_frame)
                (position, quaternion) = self.listener.lookupTransform(target_frame, source_frame, tf_latest_time)
                T_SE3 = self.listener.fromTranslationRotation(position, quaternion)

                # Transform positions since orientation of boxes are always Identity
                    #position_in_target_frame = T_SE3.dot(position_in_source_frame)
                for i in range(0, num_detected_people):
                    m = temporary_list_of_people_markers[i]
                    position_in_source_frame = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z, 1.0])
                    position_in_target_frame = T_SE3.dot(position_in_source_frame)

                    # Update Marker Position
                    m.pose.position.x, m.pose.position.y, m.pose.position.z = position_in_target_frame[0], position_in_target_frame[1], position_in_target_frame[2]

                    # print 'Person Temporary ID', i

                    # print 'Person Position In Source Frame:'
                    # print '    (x,y,z) = ', (position_in_source_frame[0], position_in_source_frame[1], position_in_source_frame[2]) 

                    # print ''
                    # print 'Person Position In Target Frame'
                    # print '    (x,y,z) = ', (temporary_list_of_people_markers[i].pose.position.x, 
                    #                          temporary_list_of_people_markers[i].pose.position.y,
                    #                          temporary_list_of_people_markers[i].pose.position.z)

                    # Visualize Transform                   
                    person_frame_name = "/person_temp_id_" + str(i)
                    br = tf.TransformBroadcaster()
                    br.sendTransform( (position_in_target_frame[0], position_in_target_frame[1], position_in_target_frame[2]),
                                     (0.0, 0.0, 0.0, 1.0),
                                     rospy.Time.now(),
                                     person_frame_name,
                                     "/my_world_neck")   

                # Process new proposed marker positions

                # Update Current List
                self.list_of_people_markers = self.people_marker_filter(temporary_list_of_people_markers)

                # Update Current Person of Interest
                #self.person_of_interest = 

                self.last_time_people_detected = rospy.Time.now().to_sec()

            except (tf.LookupException, tf.ConnectivityException):
                print 'Person transform not available yet'

    def people_marker_filter(self, proposed_list_of_people):
        return proposed_list_of_people


    def loop(self):
        current_time = rospy.Time.now().to_sec()
        print 'Last time people were detected', current_time - self.last_time_people_detected 
        print 'num of people ', len(self.list_of_people_markers)

        if ((current_time - self.last_time_people_detected) > self.time_detection_threshold):
            self.list_of_people_markers = []
            self.person_of_interest = np.array([1,0,0])

#        return