#!/usr/bin/env python

# World Time 
from datetime import datetime

# Math Dependencies
import numpy as np


experiment_name = "kinematic_control"
current_datetime = str(datetime.now())
current_datetime_no_space = ""

# for string in current_datetime.split(' '):
# 	current_datetime_no_space = current_datetime_no_space + "_" + string

savefile_name = experiment_name + current_datetime_no_space

f = open(savefile_name + '.csv', 'a')


h = [1, 2, 3, 4, 5]

# time priority head_des:x,y,z 
# 			  eye_des:x,y,z 
# 			  cur_head_x,y,z, 
# 			  cur_eye_x,y,z, 
# 			  x,y,z_error, 
# 			  head_ori_error, 
# 			  eye_ori_error,  
# 			  h1 h2 h3  
#			  q0 q1 q2 q3 q4 q5 q6
# 			  dq0 dq1 dq2 dq3 dq4 dq5 dq6
# 			  rank_task1
# 			  rank_task2

print str(datetime.now())
f.write(str(savefile_name) + "\n")
f.write(str(h[0]))
f = open(savefile_name + '.csv', 'a')
f.write(str(h[0]))
