#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped


global MAX_marker_dist, DREAMER_MARKER_ID, TRACKER_MARKER_ID, DREAMER_MARKER_PRESENT, TIME_BEFORE_PUBLISHING_FAKE_MARKER, last_dreamer_marker_update

MAX_marker_dist = 2 #2meters

DREAMER_MARKER_ID = 4
TRACKER_MARKER_ID = 3

DREAMER_MARKER_PRESENT = False
TIME_BEFORE_PUBLISHING_FAKE_MARKER = 5.0


# Publishes a fake dreamer_marker TF frame if dreamer's marker has not been visible for more than 5 seconds
def loop():
    global DREAMER_MARKER_PRESENT, TIME_BEFORE_PUBLISHING_FAKE_MARKER, last_dreamer_marker_update
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        print 'Last known update of dreamer marker', current_time - last_dreamer_marker_update, 'seconds ago'

        if ((DREAMER_MARKER_PRESENT == False) or (current_time - last_dreamer_marker_update > TIME_BEFORE_PUBLISHING_FAKE_MARKER)):
            print 'Dreamer marker not present, publishing a fake tf'
            DREAMER_MARKER_PRESENT = False
            fake_marker_x = 0.25
            fake_marker_y = 0.25
            fake_marker_z = 0.0            
            br.sendTransform( (fake_marker_x, fake_marker_y, fake_marker_z),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "/dreamer_marker_frame", # Child Frame
                             "/camera_rgb_frame") # Parent Frame

        rate.sleep()

def ar_marker_callback(marker_data):
    global MAX_marker_dist, DREAMER_MARKER_ID, TRACKER_MARKER_ID, DREAMER_MARKER_PRESENT, TIME_BEFORE_PUBLISHING_FAKE_MARKER, last_dreamer_marker_update

    marker_id = marker_data.id
    marker_frame_id = marker_data.header.frame_id
    marker_points = marker_data.points # Each point is a small square

    (p_x, p_y, p_z) = (marker_data.pose.position.x, marker_data.pose.position.y, marker_data.pose.position.z)
    # ROS convention
    (qx, qy, qz, qw) = (marker_data.pose.orientation.x, marker_data.pose.orientation.y, marker_data.pose.orientation.z, marker_data.pose.orientation.w)

    print 'Marker (ID, Frame)', (marker_id, marker_frame_id), 'detected'
    if (marker_id == DREAMER_MARKER_ID):
        print '    Dreamer Marker Detected'
        print '        Position       (x,y,z):', (p_x, p_y, p_z)
        print '        Orientation (qx, qy, qz, qw):', (qx, qy, qz, qw)

        # Convert marker positions in optical frame to 
        # camera_rgb_optical_frame to /camera_rgb_frame
        #                   Pos z  ->  Pos x
        #                   Pos x  ->  Neg y
        #                   Pos y  ->  Neg z
        br.sendTransform( (p_z, -p_x, -p_y),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/dreamer_marker_frame", # Child Frame
                         "/camera_rgb_frame") # Parent Frame
        DREAMER_MARKER_PRESENT = True
        last_dreamer_marker_update = rospy.Time.now().to_sec()

    if (marker_id == TRACKER_MARKER_ID):
        print '    Tracker Marker Detected'
        print '        Position       (x,y,z):', (p_x, p_y, p_z)
        print '        Orientation (qx, qy, qz, qw):', (qx, qy, qz, qw)

        source_frame = "/camera_rgb_optical_frame"
#        target_frame = "/dreamer_marker_frame"
        target_frame = "/my_world_neck"
        source_frame_for_viz = target_frame
        if listener.frameExists(source_frame) and listener.frameExists(target_frame):
            pose_in_source_frame = PoseStamped()
            pose_in_source_frame.header = marker_data.header
            pose_in_source_frame.pose = marker_data.pose
            pose_in_new_frame = listener.transformPose(target_frame, pose_in_source_frame) 
            print 'hello!'


            # This frame is not really utilized. We just want to visualize
            br.sendTransform( (pose_in_new_frame.pose.position.x, 
                               pose_in_new_frame.pose.position.y, pose_in_new_frame.pose.position.z),

                                 (pose_in_new_frame.pose.orientation.x,
                                  pose_in_new_frame.pose.orientation.y,
                                  pose_in_new_frame.pose.orientation.z,
                                  pose_in_new_frame.pose.orientation.w),

                             rospy.Time.now(),
                             "test_tracker_marker_tf", # Child Frame
                             source_frame_for_viz) # Parent Frame            

            #print pose_in_new_frame


    return 

if __name__ == '__main__':
    rospy.init_node('marker_position_repeater_node')
    pub = rospy.Publisher('marker_position', Float32MultiArray, queue_size=10)
    sub = rospy.Subscriber('/visualization_marker', Marker, ar_marker_callback)

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    last_dreamer_marker_update = 0

    # If human
    #sub = rospy.Subscriber('/human_bounding_box', Marker, human_marker_callback)


    rate = rospy.Rate(100)
    loop()