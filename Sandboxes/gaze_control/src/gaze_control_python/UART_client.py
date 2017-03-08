#!/usr/bin/env python

import rospy
import numpy as np
from gaze_control.srv import HeadJointCmd, HeadJointCmdRequest

global ctrl_deq_append

new_cmd = {}


def setup_ctrl_deq_append():
	global ctrl_deq_append
	rospy.wait_for_service('ctrl_deq_append')
	try:
		ctrl_deq_append = rospy.ServiceProxy('ctrl_deq_append', HeadJointCmd)
		hjc = HeadJointCmdRequest()
		hjc.numCtrlSteps.data = 550  # 1 sec

		joints = [0, 1, 2, 3, 4, 5, 6, 7]
		rads = [2.*np.pi/180. for i in joints]		

		hjc.joint_mapping.data = joints
		hjc.q_cmd_radians.data = rads		

#		hjc.joint_mapping.data.append(1)
#		hjc.q_cmd_radians.data.append(2.*np.pi/180.)
		resp = ctrl_deq_append(hjc)

		print 'hello?'

        # resp = add_two_ints(x, y)
		return resp.success
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	print setup_ctrl_deq_append()

	hjc = HeadJointCmdRequest()
	hjc.numCtrlSteps.data = 550  # 1 sec
	hjc.joint_mapping.data.append(1)
	hjc.q_cmd_radians.data.append(2.*np.pi/180.)
   
	print ctrl_deq_append(hjc).success
