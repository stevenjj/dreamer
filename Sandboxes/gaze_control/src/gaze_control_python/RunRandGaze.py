#!/usr/bin/env python

import rospy
import numpy as np
from gaze_control.srv import HeadJointCmd, HeadJointCmdRequest
from gaze_control.srv import RunProgram, RunProgramRequest
import gaze_control




CLEAR_CTRL_DEQ_PROGRAM = 0
RANDOM_GAZE_PROGRAM = 1
REMOTE_CONTROL_PROGRAM = 2



new_cmd = {}


def setup_ctrl_deq_append():
	rospy.wait_for_service('ctrl_deq_append')
	ctrl_deq_append = None
	try:
		ctrl_deq_append = rospy.ServiceProxy('ctrl_deq_append', HeadJointCmd)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return None
	else:
		return ctrl_deq_append


def setup_run_program():
	rospy.wait_for_service('run_gaze_program')
	run_gaze_program = None
	try:
		run_gaze_program = rospy.ServiceProxy('run_gaze_program', RunProgram)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return None
	else:
		return run_gaze_program


if __name__ == "__main__":
	
	rospy.init_node('UART_client', anonymous=True)

	#ctrl_deq_append = setup_ctrl_deq_append()
	run_gaze_program = setup_run_program()



	print 'Change program to RANDOM GAZE'


	rgp = RunProgramRequest()
	rgp.new_program_num.data = RANDOM_GAZE_PROGRAM
	resp = run_gaze_program(rgp)

	print resp.success.data

	exit()