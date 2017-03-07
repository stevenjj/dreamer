#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

MAX_marker_dist = 2 #2meters

DREAMER_MARKER_ID = 4
TRACKER_MARKER_ID = 3

def loop():
    while not rospy.is_shutdown():
        rate.sleep()

def ar_marker_callback(marker_data):
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
        # Pos z -> Pos x
        # Pos x -> Neg y
        # Pos y -> Neg z

        br.sendTransform( (p_z, -p_x, -p_y),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/dreamer_marker", # Child Frame
                         "/camera_rgb_frame") # Parent Frame

    if (marker_id == TRACKER_MARKER_ID):
        print '    Tracker Marker Detected'
        print '        Position       (x,y,z):', (p_x, p_y, p_z)
        print '        Orientation (qx, qy, qz, qw):', (qx, qy, qz, qw)

    rate.sleep()

    return 

if __name__ == '__main__':
    rospy.init_node('marker_position_repeater_node')
    pub = rospy.Publisher('marker_position', Float32MultiArray, queue_size=10)
    sub = rospy.Subscriber('/visualization_marker', Marker, ar_marker_callback)

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # If human
    #sub = rospy.Subscriber('/human_bounding_box', Marker, human_marker_callback)


    rate = rospy.Rate(100)
    loop()