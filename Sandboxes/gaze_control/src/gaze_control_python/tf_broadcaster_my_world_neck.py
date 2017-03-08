#!/usr/bin/env python  
import roslib
import rospy

import tf

from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('my_world_neck_tf_broadcaster')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    trans = (0, 0, 0)
    rot = (0, 0, 0, 0)     

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:    
            (trans,rot) = listener.lookupTransform('/world', '/lower_neck_pitch_link', rospy.Time(0))        
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Send a fixed transform from world to the neck.
        # Orientation of the neck is the identity matrix
        
        #from lower neck to world:
        world_x, world_y, world_z = trans[0], trans[1], trans[2]
        br.sendTransform( (-world_x, -world_y, -world_z),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/world",
                         "/my_world_neck")


        rate.sleep()