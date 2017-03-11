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

	ctrl_deq_append = setup_ctrl_deq_append()
	run_gaze_program = setup_run_program()



	print 'Change program to REMOTE CONTROL'


	rgp = RunProgramRequest()
	rgp.new_program_num.data = REMOTE_CONTROL_PROGRAM
	resp = run_gaze_program(rgp)

	print resp.success.data



	# print 'Sleep 1 s...'
	# rospy.sleep(1)
	# print ''


	# hjc = HeadJointCmdRequest()
	# hjc.numCtrlSteps.data = 550
   
	# joints = [2]
	# rads = [10.*np.pi/180. for j in joints]

	# hjc.joint_mapping.data = joints
	# hjc.q_cmd_radians.data = rads		

	# resp = ctrl_deq_append(hjc)

	# print resp.success.data

	# print ''
	# print 'Sleep 1 s...'
	# rospy.sleep(1)
	# print ''



	# print 'Send 1 second duration, 8 joint command.'

	# hjc = HeadJointCmdRequest()
	# hjc.numCtrlSteps.data = 550
   
	# joints = [0, 1, 2, 3]
	# rads = [10.*np.pi/180. for j in joints]

	# hjc.joint_mapping.data = joints
	# hjc.q_cmd_radians.data = rads		

	# resp = ctrl_deq_append(hjc)

	# print resp.success.data

	# print ''
	# print 'Sleep 1 s...'
	# rospy.sleep(1)
	# print ''

	# print 'Try sending 10 commands in a row...'
	# for cmd_num in range(10):
	# 	print 'Cmd {}: 1 second duration, 8 joint command.'.format(cmd_num)

	# 	hjc = HeadJointCmdRequest()
	# 	hjc.numCtrlSteps.data = 550
	   
	# 	joints = [0, 1, 2, 3, 4, 5, 6, 7]
	# 	rads = [cmd_num*2.*np.pi/180. for j in joints]

	# 	hjc.joint_mapping.data = joints
	# 	hjc.q_cmd_radians.data = rads

	# 	resp = ctrl_deq_append(hjc)
	# 	print resp

	# print ''
	# print 'Sleep 2 s...'
	# rospy.sleep(2)
	# print ''

	# for cmd_num in range(10, -20, -1):
	# 	print 'Cmd {}: 91 ms duration, 8 joint command.'.format(cmd_num)

	# 	hjc = HeadJointCmdRequest()
	# 	hjc.numCtrlSteps.data = 50
	   
	# 	joints = [0, 1, 2, 3, 4, 5, 6, 7]
	# 	rads = [cmd_num*2.*np.pi/180. for j in joints]

	# 	hjc.joint_mapping.data = joints
	# 	hjc.q_cmd_radians.data = rads

	# 	resp = ctrl_deq_append(hjc)
	# 	print resp
