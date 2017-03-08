#!/usr/bin/env python
import numpy as np
import serial
import time
from collections import deque
import rospy
from gaze_control.srv import HeadJointCmd, HeadJointCmdResponse
from gaze_control.srv import RunProgram, RunProgramResponse


#-------- THREADSAFE LOCKING --------#
# from threading import Lock
# lock_deq = Lock()
# blocking = True
# lock_deq.acquire(blocking)
# will block if lock is already held
# lock_deq.release()




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
CONFIRM_TIMEOUT = 200./CTRL_FREQ  # Allow up to 10 control loops to confirm a message before timeout


# -------- CONFIG FILE from Microcontroller --------#
JOINTS_MIN = {0: 4490, 1: 7100,  2: 4035, 3: 11210, 4: 10035, 5: 105, 6: 8100, 7: 300, 8: 640, 9: 1950, 10: 890, 11: 1000}
JOINTS_MAX = {0: 7325, 1: 14675, 2: 5610, 3: 13505, 4: 12350, 5: 3305, 6: 11305, 7: 15420, 8: 14350, 9: 16080, 10: 14620, 11: 15800}
JOINTS_CEN = {0: 6500, 1: (JOINTS_MIN[1]+JOINTS_MAX[1])/2, 2: (JOINTS_MIN[2]+JOINTS_MAX[2])/2,
              3: 12400,  4: 11300, 5: 1790, 6: 9625, 7: 11000, 8: (JOINTS_MIN[8]+JOINTS_MAX[8])/2,
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
    
    start_time = time.time()

    ser.write(msg_to_send)
    msg_recieved = ser.readline()

    elapsed_time = time.time() - start_time


    if msg_to_send.strip() != msg_recieved.strip():
        msg_deq.append( 'WARNING: Unexpected message recieved: ' + str(msg_recieved) + ', wait_time = ' + str(elapsed_time))
        return False
    else:
        msg_deq.append( 'Message sent and confirmed: ' + msg_recieved.strip() +
                             ', ' + decode_message(msg_recieved.strip()) ', wait_time = ' + str(elapsed_time) )
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









def check_for_mailbox_free_signal():
    global ser, msg_deq

    start_time = time.time()
    msg_recieved = (ser.readline()).strip()
    elapsed_time = time.time() - start_time

    if msg_recieved == 'G':
        msg_deq.append('Correct signal recieved: ' +str(msg_recieved) + ', wait_time = ' + str(elapsed_time))
        return True
    elif msg_recieved == 'GG':
        msg_deq.append('WARNING: speed limit exceeded!'+ str(msg_recieved) + ', wait_time = ' + str(elapsed_time))
        return True
    elif msg_recieved != '' :
        msg_deq.append( "WARNING: unexpected message recieved: " + str(msg_recieved) + ', wait_time = ' + str(elapsed_time) )
        return False
    else:
        msg_deq.append( 'Timeout, wait_time = ' + str(elapsed_time))
        return False


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


def run_program_svc_callback(req):
    global prog_request, ctrl_deq
    resp = RunProgramResponse()

    if req.new_program_num.data == CLEAR_CTRL_DEQ_PROGRAM:
        ctrl_deq.clear()
        prog_request = req.new_program_num.data
        resp.success.data = True
    elif req.new_program_num.data == RANDOM_GAZE_PROGRAM or req.new_program_num.data == REMOTE_CONTROL_PROGRAM:
        prog_request = req.new_program_num.data
        resp.success.data = True
    else:
        resp.success.data = False   # Bad program number

    return resp







def print_avail_debug_statements(deq):
    global msg_deq
    #---------- "less-intrusive" msg printing ----------#
    try:
        msg = msg_deq.popleft()    # try to get next msg we recieved
    except IndexError:
        msg = None                 # if msg_deq is empty: msg = None
   

    while msg:
        print msg
        try:
            msg = msg_deq.popleft()    # try to get next msg we recieved
        except IndexError:
            msg = None                 # if msg_deq is empty: msg = None




#-------- GLOBAL VARIABLES --------#
global ser, ctrl_deq, msg_deq, prog_request
ctrl_deq = deque()
msg_deq = deque()

# ctrl_deq will hold tuples like:  next_cmd = ( int(numCtrlSteps) , dict(joint_position: joint_cmd_rad) )



####################################
########    MAIN PROGRAM    ########
####################################
if __name__ == '__main__':
    

    rospy.init_node('UART_coms', anonymous=True)
    svc1 = rospy.Service('ctrl_deq_append', HeadJointCmd, handle_ctrl_deq_append)
    svc2 = rospy.Service('run_gaze_program', RunProgram, run_program_svc_callback)
    r = rospy.Rate(1000) # 1 kHz


    # Set Initial State
    prog_request = CLEAR_CTRL_DEQ_PROGRAM
    current_program = RANDOM_GAZE_PROGRAM
    cmd_send_success = True
    emb_mailbox_free = True
    next_cmd = None



    init_serial_port()



    debug_block = -1


    ####################################
    ########      MAIN LOOP     ########
    ####################################
    


    while not rospy.is_shutdown():


        #########################################
        ####   CHANGE PROGRAM IF NECESSARY   ####
        #########################################
        
        # this will block execution until the program is successfully changed
        if current_program != prog_request:
            if debug_block != 0:
                msg_deq.append( 'Switch to block 0')
                debug_block = 0


            if prog_request == CLEAR_CTRL_DEQ_PROGRAM:

                # Reset State
                cmd_send_success = True
                emb_mailbox_free = True
                next_cmd = None

                # Reenter current program
                prog_request = current_program
                rospy.sleep(0.5)
                clear_serial_buffer()


            else:
                # Send Run Program message to embedded controller
                msg_send_success = send_run_program_msg(prog_request)

                while not msg_send_success:
                    print_avail_debug_statements(msg_deq)
                    if prog_request == RANDOM_GAZE_PROGRAM:
                        rospy.logerr("Could not start random gaze program in embedded controller.")
                    if prog_request == REMOTE_CONTROL_PROGRAM:
                        rospy.logerr("Could not start remote control program in embedded controller.")
                    
                    rospy.sleep(0.5)
                    clear_serial_buffer()

                    msg_send_success = send_run_program_msg(prog_request)


                if msg_send_success:
                    current_program = prog_request
                    if prog_request == RANDOM_GAZE_PROGRAM:
                        msg_deq.append("Started RANDOM GAZE program in embedded controller.")

                    if prog_request == REMOTE_CONTROL_PROGRAM:
                        msg_deq.append("Started REMOTE CONTROL program in embedded controller.")


        if current_program == REMOTE_CONTROL_PROGRAM:


            ##########################################
            ####  GET NEXT COMMAND FROM CTRL DEQ  ####
            ##########################################

            # only get next command if: 
            #    last command was sent successfully and embedded controller can accept another one
            if cmd_send_success and emb_mailbox_free:
                if debug_block != 1:
                    msg_deq.append( 'Switch to block 1')
                    debug_block = 1

                try:
                    next_cmd = ctrl_deq.popleft()   # try to get next cmd to send (threadsafe)
                except IndexError:
                    next_cmd = None                 # if ctrl_deq is empty: next_cmd = None
                else:
                    cmd_send_success = False        # executes if next_cmd is successfully retrieved from ctrl_deq



            ##########################################
            ####  SEND COMMAND TO EMB CONTROLLER  ####
            ##########################################

            # if a cmd was available in the ctrl_deq and embedded controller can accept another one
            if next_cmd and emb_mailbox_free :
                if debug_block != 2:
                    msg_deq.append('Switch to block 2')
                    debug_block = 2
                
                msg_send_success = True

                for joint_num, joint_cmd_rad in next_cmd[1].iteritems():
                    # Stop sending if a message fails to send properly (exploiting short-circuit behavior)
                    msg_send_success = msg_send_success and send_joint_msg( joint_num , joint_cmd_rad )  

                # this only tries to send_start_joint_cmd_mvt() 
                #    if msg_send_success is true (exploiting short-circuit behavior)
                msg_send_success = msg_send_success and send_move_length_msg( next_cmd[0] )   \
                                                    and send_start_joint_cmd_mvt()

                if msg_send_success:
                    cmd_send_success = True
                    emb_mailbox_free = False
                else:
                    clear_serial_buffer()
                    msg_deq.append('Transmission fail. Clear read buffer...')


            ##########################################
            ####   WAIT FOR MAILBOX FREE SIGNAL   ####
            ##########################################

            if cmd_send_success and (not emb_mailbox_free):
                if debug_block != 3:
                    msg_deq.append( 'Switch to block 3')
                    debug_block = 3
                emb_mailbox_free = check_for_mailbox_free_signal()


            ##########################################
            ####   PRINT COMMS AND BLOCK SEQNCE   ####
            ##########################################

            print_avail_debug_statements(msg_deq)


        #r.sleep()  # usually it wont sleep because this loop takes longer than 1 ms


