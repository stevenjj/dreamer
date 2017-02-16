#!/usr/bin/env python
import rospy
import time

def time_test():
    rospy.init_node('timer', anonymous=True)
    rate = rospy.Rate(100) # 1hz

    ROS_start_time = rospy.Time.now().to_sec()
    system_start_time = time.time()

    while not rospy.is_shutdown():
        ROS_current_time = rospy.Time.now().to_sec() - ROS_start_time
        system_time = time.time() - system_start_time
        print "ROS time (sec): ", ROS_current_time, "System time (sec):", system_time, "delta_t:", ROS_current_time - system_time
        rate.sleep()

if __name__ == '__main__':
    try:
	   time_test()
    except rospy.ROSInterruptException:
        pass