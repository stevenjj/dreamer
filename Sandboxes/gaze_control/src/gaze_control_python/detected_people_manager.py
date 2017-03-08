#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat

import tf

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

DIST_TO_EYE = 0.1

class Person():
    def __init__(self, xyz_pos, eye_offset=np.array([0,0,0])):
        self.position = xyz_pos # np.array([x, y, z])
        self.eye_position = xyz_pos + eye_offset - np.array([0,0, DIST_TO_EYE])
        self.last_time_updated = rospy.Time.now().to_sec()

        self.average_window = 5
        self.window_index = 0

        self.window_center_positions = np.array([self.position for i in range(self.average_window)])
        self.window_eye_positions = np.array([self.eye_position for i in range(self.average_window)])

    def update(self, xyz_pos, eye_offset=np.array([0,0,0])):
        self.last_time_updated = rospy.Time.now().to_sec()        
        self.position = xyz_pos
        self.eye_position = xyz_pos + eye_offset - np.array([0,0, DIST_TO_EYE])

        # Identify current window index to update
        # Make sure to reset the window index
        if (self.window_index % self.average_window == 0):
            self.window_index
        self.window_index = (self.window_index + 1) % self.average_window

        # Update window entry 
        self.window_center_positions[self.window_index] = self.position
        self.window_eye_positions[self.window_index] = self.eye_position

        # Update filtered positions
        self.position = np.average(self.window_center_positions, axis=0)
        self.eye_position = np.average(self.window_eye_positions, axis=0)



class Detected_People_Manager():
    def __init__(self):
        self.list_of_people = [] # List of Person() objects 

        self.list_of_people_markers = []
        self.subscriber = rospy.Subscriber('/human_boxes_3D', MarkerArray, self.detected_callback)

        self.last_time_people_detected = 0 
        self.time_detection_threshold = 1 # Wait 1 second before 
        self.listener = tf.TransformListener()


    def detected_callback(self, markerArray):
        self.last_time_people_detected = rospy.Time.now().to_sec() 
        temporary_list_of_people_markers = markerArray.markers
        num_detected_people = len(temporary_list_of_people_markers)
        #print 'Number of Detected People:', num_detected_people

        if not(num_detected_people > 0):
            return

        source_frame = "/camera_rgb_optical_frame"
        target_frame = "/my_world_neck"		 
        if self.listener.frameExists(source_frame) and self.listener.frameExists(target_frame):
            try:
                # Get T_SE3 Transform
                tf_latest_time = self.listener.getLatestCommonTime(source_frame, target_frame)
                #(position, quaternion) = self.listener.lookupTransform(target_frame, source_frame, tf_latest_time)
                (position, quaternion) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))#tf_latest_time)                
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
                    # person_frame_name = "/person_temp_id_" + str(i)
                    # br = tf.TransformBroadcaster()
                    # br.sendTransform( (position_in_target_frame[0], position_in_target_frame[1], position_in_target_frame[2]),
                    #                  (0.0, 0.0, 0.0, 1.0),
                    #                  rospy.Time.now(),
                    #                  person_frame_name,
                    #                  "/my_world_neck")   

                # Process new proposed marker positions


                self.list_of_people_markers = temporary_list_of_people_markers
                self.update_list_of_people(temporary_list_of_people_markers)
                self.last_time_people_detected = rospy.Time.now().to_sec()


                for i in range(0, len(self.list_of_people)):
                    # Visualize Transform                   
                    person_frame_name = "/person_id_" + str(i)

                    person_position = self.list_of_people[i].position
                    person_tuple_position = (person_position[0], person_position[1], person_position[2])

                    br = tf.TransformBroadcaster()
                    br.sendTransform( person_tuple_position,
                                     (0.0, 0.0, 0.0, 1.0),
                                     rospy.Time.now(),
                                     person_frame_name,
                                     "/my_world_neck")   

            except (tf.LookupException, tf.ConnectivityException):
                print 'Person transform not available yet'




    # Identify whether the detected people are already in the robot's belief or not
    def update_list_of_people(self, proposed_list_of_people_markers):
        proposed_id_to_stored_person_id = {} # map id of proposed_list_of_people to id of people in robot's belief
        #this proposed_list_of_people id to current group of people

        # For each person in our robot's belief of people
        for i in range(0, len(self.list_of_people)):
            min_distance = 10000.00 # Some high number in meters
            index_of_proposed_person = 0
            new_xyz_pos = self.list_of_people[i].position
            for j in range(0, len(proposed_list_of_people_markers)):
                # Check if this person has already been given an id
                if j in proposed_id_to_stored_person_id:
                    continue
                else:
                    # Find which marker i is the closest to the person 
                    person_marker = proposed_list_of_people_markers[i]
                    person_xyz_pos = np.array([person_marker.pose.position.x, person_marker.pose.position.y, person_marker.pose.position.z]) 
                    person_position_in_belief = self.list_of_people[i].position

                    distance = np.linalg.norm((person_position_in_belief - person_xyz_pos))
                    #print 'Distance between proposed and belief', distance

                    if distance < min_distance:
                        min_distance = distance
                        index_of_proposed_person = j
                        new_xyz_pos = person_xyz_pos

            # If there are more people in our belief, stop looking for assignments for the proposed people
            if (i+1) > len(proposed_list_of_people_markers):
                break

            # Otherwise,                
            # The proposed person must be the person in our belief
            # Add this to the dictionary
            proposed_id_to_stored_person_id[index_of_proposed_person] = i
            # Update position of the person in our belief
            eye_offset = np.array([0,0,proposed_list_of_people_markers[index_of_proposed_person].scale.y/2.0])
            self.list_of_people[i].update(new_xyz_pos, eye_offset)
            #print 'updated!'

        # Go through the proposed list of people again and see if they have been assigned.
        for i in range(0, len(proposed_list_of_people_markers)):
            # If this person has not been assigned, add them as another person
            if not(i in proposed_id_to_stored_person_id):
                person_marker = proposed_list_of_people_markers[i]
                person_xyz_pos = np.array([person_marker.pose.position.x, person_marker.pose.position.y, person_marker.pose.position.z]) 
                eye_offset = np.array([0,0,proposed_list_of_people_markers[i].scale.y/2.0])
                new_person = Person(person_xyz_pos, eye_offset)
                self.list_of_people.append(new_person)




    def loop(self):
        current_time = rospy.Time.now().to_sec()
        #print 'Last time people were detected', current_time - self.last_time_people_detected 
        #print '     Num of people ', len(self.list_of_people)

        if ((current_time - self.last_time_people_detected) > self.time_detection_threshold):
            self.list_of_people = [] # Remove all people in our belief

        return