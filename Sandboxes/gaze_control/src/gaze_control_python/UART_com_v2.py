#!/usr/bin/env python
import numpy as np
import serial
import time
from collections import deque
import rospy
from gaze_control.srv import HeadJointCmd, HeadJointCmdResponse
from gaze_control.srv import RunProgram, RunProgramResponse




#------ DEFINE COMS PROTOCOL ------#
MOVE_LENGTH_REGISTER_ADDRESS = 13
UPDATE_POSITION_REGISTER_ADDRESS = 15
RUN_PROGRAM_REGSITER_ADDRESS = 17
LIGHT_COLOR_REGSITER_ADDRESS = 19

CLEAR_CTRL_DEQ_PROGRAM = 0
RANDOM_GAZE_PROGRAM = 1
REMOTE_CONTROL_PROGRAM = 2

CTRL_FREQ = 550. # Embedded controller loop rate in Hz
NUM_DOFS = 12
CONFIRM_TIMEOUT = 60./CTRL_FREQ  # Send commands at 10 Hz


# -------- CONFIG FILE from Microcontroller --------#
JOINTS_MIN = {0: 4490, 1: 7100,  2: 4035, 3: 11210, 4: 10035, 5: 105, 6: 8100, 7: 750, 8: 1640, 9: 1160, 10: 600, 11: 1010}
JOINTS_MAX = {0: 7325, 1: 14675, 2: 5610, 3: 13505, 4: 12350, 5: 3305, 6: 11305, 7: 15300, 8: 15470, 9: 15380, 10: 14480, 11: 15340}
JOINTS_CEN = {0: 6500, 1: (JOINTS_MIN[1]+JOINTS_MAX[1])/2, 2: (JOINTS_MIN[2]+JOINTS_MAX[2])/2,
              3: 12400,  4: 11300, 5: 1790, 6: 9625, 7: 10500, 8: (JOINTS_MIN[8]+JOINTS_MAX[8])/2,
              9: (JOINTS_MIN[9]+JOINTS_MAX[9])/2, 10: (JOINTS_MIN[10]+JOINTS_MAX[10])/2,
              11: (JOINTS_MIN[11]+JOINTS_MAX[11])/2}

ENC_MAX = 16384 # (2^14)
ENC_MIN = 0



# -------- HELPER FUNCTIONS --------#
def bound_joint_num(joint_num):
    if joint_num < 0:
        return 0
    elif joint_num > 11:
        return 11
    else:
        return joint_num

def bound_enc_cmd(joint_num, raw_cmd):
  if (raw_cmd > JOINTS_MAX[joint_num]):
      return JOINTS_MAX[joint_num]
  elif (raw_cmd < JOINTS_MIN[joint_num]):
      return JOINTS_MIN[joint_num]
  else:
      return raw_cmd

def degs_to_rads(deg):
    return deg * (np.pi / 180.0)

def rads_to_deg(rads):
    return rads * (180.0 / np.pi)

def enc_to_rads(enc):
    return  enc * (2*np.pi/( (float)(ENC_MAX - ENC_MIN) ) ) #rads/encoder

def rads_to_enc(rads):
    return int(np.floor(rads * (float) ((ENC_MAX - ENC_MIN) / (2*np.pi))))

def joint_cmd_to_enc_cmd(joint_num, des_joint_rads):
  raw_cmd = JOINTS_CEN[joint_num] + rads_to_enc(des_joint_rads)
  return bound_enc_cmd(joint_num, raw_cmd)




#-------- INIT SERIAL PORT -------#

def init_serial_port_name(port_name):
    global ser
    ser = serial.Serial(port_name, 115200, parity='N') # Linux use ls /dev/tty*
    # ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N') # Mac use ls /dev/tty.*
    # ser = serial.Serial('COM14', 115200, parity='N') # Windows

    ser.timeout = CONFIRM_TIMEOUT




def init_serial_port():
    ser_port_initialized = False

    errorlist = []
    if not ser_port_initialized:            # try ACM0
        port_name = '/dev/ttyACM0'
        try:
            init_serial_port_name(port_name)
        except serial.SerialException as e:
            errorlist.append("Could not open serial port: {}".format(e))
            #rospy.sleep(0.1)
        else:
            print 'Serial port initialized: ' + port_name
            ser_port_initialized = True


    if not ser_port_initialized:            # try ACM1
        port_name = '/dev/ttyACM1'
        try:
             init_serial_port_name(port_name)
        except serial.SerialException as e:
            errorlist.append("Could not open serial port: {}".format(e))
        else:
            print 'Serial port initialized: ' + port_name
            ser_port_initialized = True

    if not ser_port_initialized:
        for e in errorlist:
            rospy.logerr(e)                 # only print error message if neither name works
        exit()




# -------- MESSAGE FUNCTIONS --------#

def create_binary_msg(register, register_value):
    # ------ FORMAT MESSAGE -------#
    address = format(register, '07b') # place joint number (0-11) in first 7 bits
    value = format(register_value, '024b') # place desired joint pos in encoder ticks units in the next 24 bits
    bit_31_cmd = address + value # combine to form 31 bits represented as a string
    bit_sum = sum([int(i) for i in bit_31_cmd if int(i) == 1])
    parity = str(  bit_sum % 2) # calculate parity of 31 bits
    final_cmd = bit_31_cmd + parity # finalize command to form 32 bits represented as a string
    #  address            value                  parity
    # 0000 000 | 0 0000 0000 0000 0000 0000 000 |   0

    binary_form = final_cmd
    # hex_form = hex(int(final_cmd, 2))
    integer_form = int(final_cmd, 2)
    msg_to_send = str(integer_form) + "\r" # message to send is the integer value.
    return msg_to_send



def decode_message(dec_msg):
    integer_form = int(dec_msg.strip())
    binary_form = format(integer_form,'032b')
    parity =   int(binary_form[-1]    )
    value =    int(binary_form[7:-1],2)
    register = int(binary_form[0: 7],2)

    return 'R = {}, V = {}, P = {}'.format(register, value, parity)





def send_and_confirm(msg_to_send):
    global ser, msg_deq
    
    ser.write(msg_to_send)
    rospy.sleep(0.002)
    # msg_recieved = ser.readline()
    msg_deq.append("Sent msg: "+decode_message(msg_to_send))

    return True



def clear_serial_buffer():
    global ser
    ser.flushInput()



def send_run_program_msg(program_num):
    register = RUN_PROGRAM_REGSITER_ADDRESS
    msg_to_send = create_binary_msg(register, program_num)

    return send_and_confirm(msg_to_send)



def send_joint_msg(joint_num, joint_value_rads):
    joint_num = bound_joint_num(joint_num)
    des_enc_cmd = joint_cmd_to_enc_cmd(joint_num, joint_value_rads)

    register = joint_num
    register_value = des_enc_cmd

    msg_to_send = create_binary_msg(register, register_value)
    
    return send_and_confirm(msg_to_send)



def send_ear_color_msg(color_value):
    register = LIGHT_COLOR_REGSITER_ADDRESS
    msg_to_send = create_binary_msg(register, color_value)
    
    return send_and_confirm(msg_to_send)



def send_start_joint_cmd_mvt():
    register = UPDATE_POSITION_REGISTER_ADDRESS
    msg_to_send = create_binary_msg(register, 0)
    
    return send_and_confirm(msg_to_send)



def send_move_length_msg(num_ctrl_cycles):  # ctrl loop is 550 Hz => 0.0018s
    register = MOVE_LENGTH_REGISTER_ADDRESS
    msg_to_send = create_binary_msg(register, num_ctrl_cycles)
    
    return send_and_confirm(msg_to_send)
















def handle_ctrl_deq_append(req):
    global ctrl_deq
    new_cmd = {}
    
    for joint_num, joint_cmd_rad in zip( req.joint_mapping.data , req.q_cmd_radians.data ):
        new_cmd[joint_num] = joint_cmd_rad
        
    ctrl_tup = ( req.numCtrlSteps.data , new_cmd )
    
    ctrl_deq.append( ctrl_tup )   # threadsafe

    resp = HeadJointCmdResponse()
    resp.success.data = True
    return resp 

    # Could use resp to ensure the fifo does not get too large:
    #   Set a maximum FIFO size.  If appending would exceed fifo size return False
    #   This would ensure that the sender does not get too far ahead of real time





def run_program_svc_callback(req):
    global prog_request, ctrl_deq
    resp = RunProgramResponse()

    if req.new_program_num.data == CLEAR_CTRL_DEQ_PROGRAM:
        ctrl_deq.clear()
        resp.success.data = True
    elif req.new_program_num.data == RANDOM_GAZE_PROGRAM or req.new_program_num.data == REMOTE_CONTROL_PROGRAM:
        prog_request = req.new_program_num.data
        resp.success.data = True
    else:
        resp.success.data = False   # Bad program number

    return resp



def print_avail_debug_statements(deq):
    #---------- "less-intrusive" msg printing ----------#
    try:
        msg = deq.popleft()    # try to get next msg we recieved
    except IndexError:
        msg = None                 # if msg_deq is empty: msg = None

    while msg:
        print msg
        try:
            msg = deq.popleft()    # try to get next msg we recieved
        except IndexError:
            msg = None                 # if msg_deq is empty: msg = None




#-------- GLOBAL VARIABLES --------#
global ser, ctrl_deq, msg_deq, prog_request
ctrl_deq = deque()
msg_deq = deque()

# ctrl_deq will hold tuples like:  next_cmd = ( int(numCtrlSteps) , dict(joint_position: joint_cmd_rad) )


# STATES
WAIT_FOR_WELCOME = 0
WAIT_FOR_EMB_SIG = 1
WAIT_FOR_CMDS    = 2
SENDING_CMDS     = 3
RESET            = 4
CHANGE_PROGRAMS  = 5
ACK_PROG_CHG     = 6
ACK_E_STOP       = 7


STATES = [WAIT_FOR_WELCOME, WAIT_FOR_EMB_SIG, WAIT_FOR_CMDS, \
            SENDING_CMDS, RESET, CHANGE_PROGRAMS, ACK_PROG_CHG, ACK_E_STOP]







####################################
########    MAIN PROGRAM    ########
####################################
if __name__ == '__main__':
    

    rospy.init_node('UART_coms', anonymous=True)
    svc1 = rospy.Service('ctrl_deq_append', HeadJointCmd, handle_ctrl_deq_append)
    svc2 = rospy.Service('run_gaze_program', RunProgram, run_program_svc_callback)
    r = rospy.Rate(1000) # 1 kHz

    init_serial_port()

    prog_request    = RANDOM_GAZE_PROGRAM
    current_program = RANDOM_GAZE_PROGRAM

    next_state      = WAIT_FOR_WELCOME

    next_cmd            = None
    last_go_time        = None
    first_attempt_time  = None

    
    print "Press the Reset Button to continue."
    while next_state == WAIT_FOR_WELCOME:
        msg_recieved = (ser.readline()).strip()
        if msg_recieved == 'Welcome':
            msg_deq.append('Embedded controller was reset. RANDOM GAZE program started.')
            next_state = RESET
        elif msg_recieved != '':
            print ("Msg recieved: '"+ msg_recieved+ "', but waiting for 'Welcome'. Press the Reset Button.")


    while not rospy.is_shutdown():


        # Print cool stuff
        print_avail_debug_statements(msg_deq)

        # Update state
        state = next_state

        # Program change will override other state transitions
        if prog_request != current_program:  
            state = CHANGE_PROGRAMS






        ############################
        if state == RESET:
            last_go_time       = None
            first_attempt_time = None

            fifo_size = len(ctrl_deq)
            if fifo_size:
                msg_deq.append('RESET: Fifo size is currently '+str(fifo_size)+'.  Attempting to clear it.\n'+ \
                               'Please stop sending data to continue. Sleeping for 3 sec...')
                print_avail_debug_statements(msg_deq)
                ctrl_deq.clear()
                rospy.sleep(3)
            else:
                msg_deq.append('Fifo has been empty for at least three seconds...')
                if current_program == REMOTE_CONTROL_PROGRAM:
                    msg_deq.append('REMOTE CONTROL program will go home before executing commands.')
                next_state = ACK_PROG_CHG





        ############################
        if state == CHANGE_PROGRAMS:

            if prog_request != current_program:

                msg_send_success = send_run_program_msg(prog_request)

                while not msg_send_success:
                    
                    if prog_request == RANDOM_GAZE_PROGRAM:
                        msg_deq.append("Could not start RANDOM GAZE program in embedded controller.")
                    if prog_request == REMOTE_CONTROL_PROGRAM:
                        msg_deq.append("Could not start REMOTE CONTROL program in embedded controller.")
                    msg_send_success = send_run_program_msg(prog_request)

                # Once successful...
                current_program = prog_request

                if prog_request == RANDOM_GAZE_PROGRAM:
                    msg_deq.append("Started RANDOM GAZE program in embedded controller.")
                    next_state = ACK_PROG_CHG

                if prog_request == REMOTE_CONTROL_PROGRAM:
                    msg_deq.append("Started REMOTE CONTROL program in embedded controller.")
                    next_state = RESET

            else:
                if current_program == RANDOM_GAZE_PROGRAM:
                    msg_deq.append('Skipping change program. Already running RANDOM GAZE program.')
                if current_program == REMOTE_CONTROL_PROGRAM:
                    msg_deq.append('Skipping change program. Already running REMOTE CONTROL program.')






        ############################
        if state == ACK_PROG_CHG:

            msg_recieved = (ser.readline()).strip()

            if msg_recieved == 'R':
                if current_program == REMOTE_CONTROL_PROGRAM:
                    next_state = WAIT_FOR_EMB_SIG
                elif current_program == RANDOM_GAZE_PROGRAM:
                    next_state = ACK_E_STOP

            elif msg_recieved == 'Welcome':
                msg_deq.append('Embedded controller was reset. RANDOM GAZE program started.')
                current_program = RANDOM_GAZE_PROGRAM
                prog_request = RANDOM_GAZE_PROGRAM
                next_state = RESET

            elif msg_recieved == 'G':
                msg_deq.append("You should never be here. Recieved 'G' while waiting for a program change acknowledgment ('R').")








        ############################
        if state == ACK_E_STOP:
            if current_program == REMOTE_CONTROL_PROGRAM:
                last_go_time       = None
                first_attempt_time = None
                next_cmd           = None

                fifo_size = len(ctrl_deq)
                if fifo_size:
                    msg_deq.append('E-Stopped: Fifo size is currently '+str(fifo_size)+'.  Attempting to clear it.\n'+ \
                                   'Please stop sending data to continue. Sleeping for 3 sec...')
                    print_avail_debug_statements(msg_deq)
                    ctrl_deq.clear()
                    rospy.sleep(3)
                else:
                    msg_deq.append('Fifo has been empty for at least three seconds...')
                    msg_deq.append('REMOTE CONTROL program will go home before executing commands.')
                    next_state = WAIT_FOR_EMB_SIG

            elif current_program == RANDOM_GAZE_PROGRAM:

                msg_recieved = (ser.readline()).strip()

                if msg_recieved == 'R':
                    next_state = ACK_E_STOP                     # Loop can only be broken by changing program (via prog_request)

                elif msg_recieved == 'Welcome':
                    msg_deq.append('Embedded controller was reset. RANDOM GAZE program started.')
                    current_program = RANDOM_GAZE_PROGRAM
                    prog_request = RANDOM_GAZE_PROGRAM
                    next_state = RESET










        ############################
        if state == WAIT_FOR_EMB_SIG:
            
            msg_recieved = (ser.readline()).strip()

            if msg_recieved == 'G':

                # Measure Rate
                go_time = time.time()

                if not last_go_time:
                    msg_deq.append("Recieved first 'GO' signal.")
                else:
                    elapsed_time = (go_time - last_go_time)
                    go_rate = 1./elapsed_time
                    elapsed_time *= 1000.
                    msg_deq.append("Recieved 'GO' signal. Time since last = "+ format(elapsed_time, '6.2f')+ ' ms ('+format(go_rate,'6.2f')+' Hz). FIFO size = '+format( len(ctrl_deq),'3'))
                last_go_time = go_time

                next_state = WAIT_FOR_CMDS


            elif msg_recieved == 'Welcome':
                msg_deq.append('Embedded controller was reset. RANDOM GAZE program started.')
                current_program = RANDOM_GAZE_PROGRAM
                prog_request = RANDOM_GAZE_PROGRAM
                next_state = RESET
            elif msg_recieved == 'R':
                msg_deq.append("System was E-Stopped...")
                next_state = ACK_E_STOP
            elif msg_recieved == '' and next_cmd == None:   # Dont change state in this case
                msg_deq.append("Timeout while waiting for 'GO' signal.")
                # next_state = WAIT_FOR_EMB_SIG         
            elif msg_recieved == '' and next_cmd != None:
                msg_deq.append("Timeout while waiting for 'GO' signal. Skipping to next command...")
                next_state = WAIT_FOR_CMDS
            else:
                msg_deq.append("You shouldnt be here. Are you?")






        ############################
        if state == WAIT_FOR_CMDS:
            if not first_attempt_time:
                first_attempt_time = time.time()

            try:
                next_cmd = ctrl_deq.popleft()   # try to get next cmd to send (threadsafe)
            except IndexError:
                next_cmd = None                 # if ctrl_deq is empty: next_cmd = None
                ser.timeout = 0
                msg_recieved = (ser.readline()).strip()
                if msg_recieved == 'Welcome':
                    msg_deq.append('Embedded controller was reset. RANDOM GAZE program started.')
                    current_program = RANDOM_GAZE_PROGRAM
                    prog_request = RANDOM_GAZE_PROGRAM
                    next_state = RESET
                elif msg_recieved == 'R':
                    msg_deq.append("System was E-Stopped...")
                    next_state = ACK_E_STOP
                ser.timeout = CONFIRM_TIMEOUT
            else:
                elapsed_time = 1000.*(time.time() - first_attempt_time)
                msg_deq.append("Took "+format(elapsed_time,'6.2f')+" ms to get next command from FIFO.")
                first_attempt_time = None
                next_state = SENDING_CMDS       # executes if next_cmd is successfully retrieved from ctrl_deq









        ############################
        if state == SENDING_CMDS:
            # this_time = time.time()
            # if last_time:
            #     send_rate = 1./(this_time - last_time)
            #     print 'Send rate = ', send_rate, ' hz, FIFO size = ', len(ctrl_deq)
            # last_time = this_time
            
            msg_send_success = True

            for joint_num, joint_cmd_rad in next_cmd[1].iteritems():
                send_joint_msg( joint_num , joint_cmd_rad )  

            send_move_length_msg( next_cmd[0] )
            send_start_joint_cmd_mvt()

            elapsed_time = 1000.*(time.time() - last_go_time)
            msg_deq.append("Finished sending commands. Time since last 'GO' = "+format(elapsed_time,'6.2f')+' ms.')

            next_state = WAIT_FOR_EMB_SIG




            

        #r.sleep()  # dont need sleep due to serial synchronization


