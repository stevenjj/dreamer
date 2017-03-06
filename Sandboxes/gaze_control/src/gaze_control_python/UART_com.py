#!/usr/bin/env python
import numpy as np
import serial
import time
from collections import deque
import rospy
from gaze_control.srv import HeadJointCmd


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

RANDOM_GAZE_PROGRAM = 1
REMOTE_CONTROL_PROGRAM = 2

CTRL_FREQ = 550. # Embedded controller loop rate in Hz
NUM_DOFS = 12
CONFIRM_TIMEOUT = 10./CTRL_FREQ  # Allow up to 10 control loops to confirm a message before timeout


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




# -------- MESSAGE FUNCTIONS --------#
def init_serial_port():
    #-------- INIT SERIAL PORT -------#
    global ser
    ser = serial.Serial('/dev/ttyACM0', 115200, parity='N') # Linux use ls /dev/tty*
    # ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N') # Mac use ls /dev/tty.*
    # ser = serial.Serial('COM14', 115200, parity='N') # Windows

    ser.timeout = 1./(CTRL_FREQ*2)  # no idea if this is going to work
    # 0.0009 s


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
    hex_form = hex(int(final_cmd, 2))
    integer_form = int(final_cmd, 2)
    msg_to_send = str(integer_form) + "\r" # message to send is the integer value.
    return msg_to_send



def send_and_confirm(msg_to_send):
    global ser, empty_msg_count
    start_time = time.time()
    elapsed_time = 0.
    msg_recieved = ''

    while (elapsed_time < CONFIRM_TIMEOUT) and (msg_to_send.strip() != msg_recieved.strip()) :
        ser.write(msg_to_send)
        
        msg_recieved = ser.readline()
        elapsed_time = time.time() - start_time

        while msg_recieved == serial.to_bytes([]) :
            # empty_msg_count += 1
            msg_recieved = ser.readline()
            elapsed_time = time.time() - start_time


        # print 'empty_msg_count = ' + str(empty_msg_count)
        # print 'msg_sent = ' + str(msg_to_send)
        # print 'msg_recd = ' + str(msg_recieved)

    if msg_to_send.strip() != msg_recieved.strip():
        return False
    else:
        return True



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


def check_for_emb_signal():
    global ser
    start_time = time.time()
    elapsed_time = 0.
    msg_recieved = serial.to_bytes([])

    while elapsed_time < CONFIRM_TIMEOUT and msg_recieved == serial.to_bytes([]):
        msg_recieved = (ser.readline()).rstrip()
        elapsed_time = time.time() - start_time

    if msg_recieved == 'R':
        return True
    elif msg_recieved == 'RF':
        print 'WARNING: speed limit exceeded!'
        return True
    elif msg_recieved != serial.to_bytes([]) :
        print ("WARNING: unexpected message:" + msg_recieved)
    else:
        return False



def handle_ctrl_deq_append(req):
    global ctrl_deq
    new_cmd = {}
    for joint_num, joint_cmd_rad in zip( req.joint_mapping.data , req.q_cmd_radians.data ):
        new_cmd[joint_num] = joint_cmd_rad

    ctrl_tup = ( req.numCtrlSteps , new_cmd )
    
    ctrl_deq.append( ctrl_tup )   # threadsafe
    
    return HeadJointCmdResponse(True)



#-------- GLOBAL VARIABLES --------#

# ctrl_deq will hold tuples like:  next_cmd = ( int(numCtrlSteps) , dict(joint_position: joint_cmd_rad) )
global ser, ctrl_deq, empty_msg_count
ctrl_deq = deque()
empty_msg_count = 0



#-------- MAIN PROGRAM ------------#
if __name__ == '__main__':
    

    rospy.init_node('UART_coms', anonymous=True)
    svc = rospy.Service('ctrl_deq_append', HeadJointCmd, handle_ctrl_deq_append)


    cmd_send_success = True
    emb_que_available = True
    next_cmd = None

    try:
        init_serial_port()
    except serial.SerialException as e:
        rospy.logerr("Could not open serial port: {}".format(e))


    cmd_send_success = send_run_program_msg(REMOTE_CONTROL_PROGRAM)
    while not cmd_send_success:
        rospy.logerr("Could not start remote control program in embedded controller.")
        cmd_send_success = send_run_program_msg(REMOTE_CONTROL_PROGRAM)


    print 'Started remote control program...'


    debug_count = 0



    #-------- MAIN LOOP ------------#
    r = rospy.Rate(10) # 1 kHz
    while not rospy.is_shutdown():
        
        # only get next command if: 
        #    last command was sent successfully and embedded controller can accept another one
        if cmd_send_success and emb_que_available:
            print 'Inside Block for Step 1'
            try:
                next_cmd = ctrl_deq.popleft()   # try to get next cmd to send (threadsafe)
            except IndexError:
                next_cmd = None                 # if ctrl_deq is empty: next_cmd = None
            else:
                cmd_send_success = False        # executes if next_cmd is successfully retrieved from ctrl_deq



        # if a cmd was available in the ctrl_deq and embedded controller can accept another one
        if next_cmd and emb_que_available :
            print 'Inside Block for Step 2'
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
                emb_que_available = False


        if cmd_send_success and (not emb_que_available):
            print 'Inside Block for Step 3'
            emb_que_available = check_for_emb_signal()


        r.sleep()  # usually it wont sleep because this loop takes longer than 1 ms


