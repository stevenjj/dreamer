#!/usr/bin/env python  
import roslib
import rospy

import tf

from geometry_msgs.msg import TransformStamped

# Center of Marker to Center of Neck

if __name__ == '__main__':
    rospy.init_node('fixed_marker_to_my_world_neck_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Send a fixed transform from world to the neck.
        # Orientation of the neck is the identity matrix
        br.sendTransform((0.190, 0.0, 0.001),
                         (0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/my_world_neck",
                         "/dreamer_marker_frame")
        rate.sleep()