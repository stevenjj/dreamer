#!/usr/bin/env python
import rospy
import time
import math
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk
import dreamer_joint_publisher
#from head_kinematics import 

'''
state machine:

- idle
- go to point.
- move head circular direction

Joint Publisher object
Head Kinematics object


initialize state
loop:




Trajectory manager


'''


def calculate_dQ(J, dx):
	dQ = np.linalg.pinv(J).dot(dx)
	return dQ

def circular_trajectory(t):
    radius = 0.2
    x_offset, y_offset, z_offset = 0.25, 0, 0.01
    T = 6.0
    frequency = 1.0/T
    angular_frequency = 2*3.14*frequency
    x = x_offset
    y = y_offset + radius*np.cos(angular_frequency*t)
    z = z_offset + radius*np.sin(angular_frequency*t)	
    return np.array([x, y, z])
	# Publish x(t) markers.

# Calculate desired orientation
# Accepts a 3x1 gaze location vector and an origin vector to create the desired orientation
def calc_desired_orientation(x_gaze_loc, p_cur):
    # Calculate Desired Orientation
    z_world_hat = np.array([0,0,1])
    y_world_hat = np.array([0,1,0])    

    p_bar = x_gaze_loc - p_cur
    x_hat_d = p_bar/np.linalg.norm(p_bar)

    # Threshold before we use y_world_hat
    epsilon = 0.1 * np.pi/180.0 # degrees in radians 
    phi = np.arccos( (x_hat_d.dot(z_world_hat)) / (np.linalg.norm(x_hat_d)*np.linalg.norm(z_world_hat)) )

    z_hat_o = z_world_hat
    if (phi <= epsilon):
        z_hat_o = y_world_hat

    z_hat_d = mr.Normalize(z_hat_o - (z_hat_o.dot(x_hat_d)*x_hat_d))
#    print 'z_hat projected', z_hat_d.T

    y_hat_d = np.cross(z_hat_d, x_hat_d)

    R_desired = np.array([x_hat_d, y_hat_d, z_hat_d]).T
#    print 'Desired Orientation ', R_desired
 #   print 'x_hat', x_hat_d.T
 #   print 'y_hat', y_hat_d.T
 #   print 'z_hat', z_hat_d.T

#    print R_desired
    return R_desired

# Calculate error between current configuration and desired configuration
def orientation_error(x_gaze_loc, Q, orientation_type='head'):
    global head_kin
    J = None
    R_cur, p_cur = None, None
    # Get forward kinematics
    if (orientation_type == 'head'):
        R_cur, p_cur = head_kin.get_6D_Head_Position(Q)
    elif (orientation_type == 'right_eye'):
        R_cur, p_cur = head_kin.get_6D_Right_Eye_Position(Q)
    elif (orientation_type == 'left_eye'):        
        R_cur, p_cur = head_kin.get_6D_Left_Eye_Position(Q)
    else:
        raise 'unknown position and orientation needed'

    # Calculate desired orientation
    R_des = calc_desired_orientation(x_gaze_loc, p_cur)

    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    return quat.quat_to_wth(q_error)
   #  theta = 2*np.arccos(q_error[0])

   #  #print theta, q_error[0], q_error
   #  if (mr.NearZero(1.0 - q_error[0])):
   #      return 0, q_error[1:]

   #  factor = np.sin(theta/2.0)
   #  w_hat_x = q_error[1]/factor
   #  w_hat_y = q_error[2]/factor
   #  w_hat_z = q_error[3]/factor        

   #  angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

   # # print theta, angular_vel_hat
   #  return theta, angular_vel_hat

# define zero position error
def zero_position_error():
    return np.array([0,0,0])

def rotation_quaternion_error(R_cur, R_des):
    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))


    theta = 2*np.arccos(q_error[0])

    #print theta, q_error[0], q_error
    if (mr.NearZero(1.0 - q_error[0])):
        return 0, q_error[1:]

    factor = np.sin(theta/2.0)
    w_hat_x = q_error[1]/factor
    w_hat_y = q_error[2]/factor
    w_hat_z = q_error[3]/factor        

    angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

   # print theta, angular_vel_hat
    return theta, angular_vel_hat

# State Lists
IDLE = 0
GO_TO_POINT = 1
FOLLOWING_TRAJECTORY = 2
FINISHED_TRAJECTORY = 3
INTERRUPTED = 4

# High Level Task Lists
NO_TASK = 100
GAZE_AVERSION = 101
TASK_HIERARCHY = 102
CIRCULAR_HEAD_TRAJ = 103
TRACK_PERSON = 104
TRACK_MARKER = 105

# High Level Behavior List --- seems like this is another high level task list
DO_NOTHING = 200
ALWAYS_GAZE_AVERT = 201
ALWAYS_GAZE_FOLLOW = 202


class Dreamer_Head():
    command_once = False
    def __init__(self):
        self.behaviors = []
        self.tasks = [NO_TASK, GAZE_AVERSION, TASK_HIERARCHY, CIRCULAR_HEAD_TRAJ, TRACK_PERSON, TRACK_MARKER]
        self.states = [IDLE, GO_TO_POINT, FOLLOWING_TRAJECTORY, FINISHED_TRAJECTORY, INTERRUPTED]

        self.current_state = IDLE
        self.current_task = NO_TASK

        self.kinematics = hk.Head_Kinematics() 
        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()

        # ROS Loop details
        # Send control messages at 500hz. 10 messages to send per ROS loop so this node operates at 50Hz
        self.node_rate = 1/(500/10.0)
        self.rate = rospy.Rate(500) 
        self.ROS_start_time = rospy.Time.now().to_sec()
        self.ROS_current_time = rospy.Time.now().to_sec()

        self.traj_manager = Trajectory_Manager()

        self.task_list = [0, 1]

        # READ Current Joint Positions
        # Otherwise send HOME command

    def callback_change_state(self, string):
        #specifying subscriber callback via callback class_instance.method_name
        return 

    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list

        def joint_cmd_bound(val, jmax, jmin):
            if val >= jmax:
                return jmax
            elif (val <= jmin):
                return jmin
            else:
                return val

        for i in range(0, len(head_joint_list)):
            command = head_joint_list[i]
            joint_name = self.kinematics.Jindex_to_names[i]
            joint_max_val = self.joint_publisher.free_joints[joint_name]['max']
            joint_min_val = self.joint_publisher.free_joints[joint_name]['min']
            self.joint_publisher.free_joints[joint_name]['position'] = joint_cmd_bound(command, joint_max_val, joint_min_val)


    def task_logic(self):
        self.ROS_current_time = rospy.Time.now().to_sec()
        relative_time =  self.ROS_current_time - self.ROS_start_time
        print "ROS time (sec): ", relative_time            

        if ((relative_time > 0.1) and (self.command_once == False)):
            self.current_state = GO_TO_POINT
            # using trajectory_manager
            #   specify xyz goal point
            #   specify desired duration time
            #   save start time
            #   set current time to start time

            start_time = self.ROS_current_time
            Q_cur = self.kinematics.Jlist
#            xyz_gaze_loc = np.array([0.5, 0.5, self.kinematics.l1+0.4])
#            xyz_gaze_loc = np.array([0.3, 0.0, self.kinematics.l1+0.5])
            xyz_gaze_loc = np.array([0.4, 0.4, self.kinematics.l1+0.2])            
#            xyz_gaze_loc = np.array([0.3, 0.3, self.kinematics.l1+0.0])            

            small_delta_t = self.node_rate
            movement_duration = 5
            self.traj_manager.specify_goal(start_time, Q_cur, xyz_gaze_loc, small_delta_t, movement_duration)
            #head_joints = [np.pi/4.0, np.pi/6.0, 0, 0, 0, 0, 0]
            #self.update_head_joints(head_joints)
            self.command_once = True

        return 

    def behavior_logic(self):
        return 

    def state_logic(self):
        if (self.current_state == IDLE):
            print "STATE = Idle"
        elif (self.current_state == GO_TO_POINT):
            print "STATE = GO_TO_POINT"
            #Q_des, command_result = self.traj_manager.go_to_point()
            Q_des, command_result = self.traj_manager.go_to_point2()
            if (command_result == True):
               self.current_state = IDLE
            self.update_head_joints(Q_des)
        else:
            print "ERROR Not a valid state" 

    def loop(self):
        while not rospy.is_shutdown():
            self.task_logic()
            self.state_logic()          
            # message rate = 500 Hz, num of messages to send = 10
            # node rate = 500/10.0 = 50 hz
            for i in range(0, 10): # 
                # send message
                self.rate.sleep()   #rospy.sleep(1/500.0);    
                # Will sleep for a total of 0.02 seconds --> 50Hz
            self.joint_publisher.publish_joints()

class Trajectory_Manager():
    def __init__(self):
        self.kinematics = hk.Head_Kinematics() 

        self.start_time = 0
        self.current_traj_time = 0
        self.prev_traj_time = 0
        self.movement_duration = 1
        self.small_delta_t = 500/10.0
        self.epsilon = 0.01

        self.xyz_gaze_loc = np.array([0,0,0])

        self.theta_total_head = 0
        self.angular_vel_hat_head = np.array([0,0,0])        

        self.theta_total_right_eye = 0
        self.angular_vel_hat_right_eye = np.array([0,0,0])         
               
        self.theta_total_left_eye = 0
        self.angular_vel_hat_left_eye = np.array([0,0,0])        

        self.Q_o_at_start = self.kinematics.Jlist


    # if error < xx return 'Change State'
    def specify_goal(self, start_time, Q_cur, xyz_gaze_loc, small_delta_t, movement_duration):
        self.kinematics.Jlist = Q_cur
        self.xyz_gaze_loc = xyz_gaze_loc
        self.movement_duration = movement_duration
        self.start_time = start_time
        self.current_traj_time = start_time
        self.small_delta_t = small_delta_t       

        self.theta_total, self.angular_vel_hat = orientation_error(xyz_gaze_loc, Q_cur)
        self.theta_total_right_eye, self.angular_vel_hat_right_eye = orientation_error(xyz_gaze_loc, Q_cur, 'right_eye')
        self.theta_total_left_eye, self.angular_vel_hat_left_eye = orientation_error(xyz_gaze_loc, Q_cur, 'left_eye')

        self.Q_o_at_start = Q_cur

        return

    def min_jerk_time_scaling(self, t, delta_t):
        s_t = 0.0
        x_init, x_final = 0.0, 1.0 # Set to 0 and 1 since we want time scaling from 0 to 1
        if (t < 0):
            s_t = 0.0
        elif (t >= delta_t):
            s_t = 1.0
        else:
            s_t = x_init + (x_final-x_init)*( 10.0*((t/delta_t)**3.0) - 15.0*((t/delta_t)**4.0) + 6.0*((t/delta_t)**5.0) )
        return s_t 

    def go_to_point(self):
        self.current_traj_time = rospy.Time.now().to_sec() - self.start_time

        t = self.current_traj_time
        t_prev = self.prev_traj_time
        xyz_gaze_loc = self.xyz_gaze_loc

        # Get Current Config Q and Jacobian
        Q_cur = self.kinematics.Jlist
        J = self.kinematics.get_6D_Head_Jacobian(Q_cur)
        J = J[0:3,:] #Grab the first 3 rows        

        # If we're in motion, we do the following loop
        dt = t - t_prev
        DT = self.movement_duration

        # Calculate new desired joint position
        d_theta_error = self.theta_total*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))
        dx = d_theta_error * self.angular_vel_hat
        dq = calculate_dQ(J, dx)
        Q_des = Q_cur + dq 



        theta_error, angular_vel_hat = orientation_error(xyz_gaze_loc, Q_cur)      



        q_t_error = quat.wth_to_quat(self.angular_vel_hat, self.theta_total * self.min_jerk_time_scaling(t, DT))
        q_orig = quat.R_to_quat( self.kinematics.get_6D_Head_Position( self.Q_o_at_start  )[0] )
        q_current_desired = quat.quat_multiply(q_t_error, q_orig)

        q_c = quat.R_to_quat( self.kinematics.get_6D_Head_Position(Q_cur)[0] )
        q_feedback_error = quat.quat_multiply(q_current_desired, quat.conj(q_c))


        print 'q current_desired: ', quat.quat_to_wth(q_current_desired)
        print 'q c: ', quat.quat_to_wth(q_c)        
        print 'q feedbackerror: ', quat.quat_to_wth(q_feedback_error)


        fe_dt_theta, fe_angular_vel = quat.quat_to_wth(q_feedback_error)
        dx_fb = fe_dt_theta*fe_angular_vel
        dq_fb = calculate_dQ(J, dx_fb)

        print 'current dq', dq, 'feedback dq', dq_fb
        Q_des = Q_des + dq_fb


        print theta_error, t

        result = False
        if (t > DT):
            result = True

        self.kinematics.Jlist = Q_des
        self.prev_traj_time = t

        return Q_des, result

    def go_to_point2(self):
        self.current_traj_time = rospy.Time.now().to_sec() - self.start_time

        t = self.current_traj_time
        t_prev = self.prev_traj_time
        xyz_gaze_loc = self.xyz_gaze_loc

        # Get Current Config Q and Jacobian
        Q_cur = self.kinematics.Jlist
         # Two tasks
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)
        J_1 = J_1[0:3,:] #Grab the first 3 rows      
        J_2 = J_2[0:3,:] #Grab the first 3 rows            
        J = np.concatenate((J_1,J_2) ,axis=0)

        # If we're in motion, we do the following loop
        dt = t - t_prev
        DT = self.movement_duration

        # Calculate new desired joint position
        d_theta_error_right_eye = self.theta_total_right_eye*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))
        d_theta_error_left_eye = self.theta_total_left_eye*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))        
        dx_right_eye = d_theta_error_right_eye * self.angular_vel_hat_right_eye
        dx_left_eye = d_theta_error_left_eye * self.angular_vel_hat_left_eye        

        dx_two_tasks = np.concatenate((dx_right_eye, dx_left_eye), axis=1)

#        J = J_1        
#        dx_two_tasks = dx_right_eye

        dq = calculate_dQ(J, dx_two_tasks)
        Q_des = Q_cur + dq 


        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_gaze_loc, Q_cur, 'left_eye')

        print theta_error_right_eye, theta_error_left_eye, t
        
        result = False
        if (t > DT):
            result = True

        self.kinematics.Jlist = Q_des
        self.prev_traj_time = t

        return Q_des, result



if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    global head_kin
    head_kin = hk.Head_Kinematics() 
    kin = hk.Head_Kinematics()

    print kin.S0
    
    orientation_error(np.array([1.053,0, 0.13849]) , kin.Jlist ,'head')

    J_head = kin.get_6D_Head_Jacobian(kin.Jlist)
    J_right_eye = kin.get_6D_Right_Eye_Jacobian(kin.Jlist)		
    J_left_eye = kin.get_6D_Left_Eye_Jacobian(kin.Jlist)


    dreamer_head = Dreamer_Head()
    dreamer_head.loop()

#        rospy.init_node('head_joint_publisher')
#        custom_joint_publisher = Custom_Joint_Publisher() 
#        custom_joint_publisher.loop()
#    except rospy.ROSInterruptException:
#        pass
