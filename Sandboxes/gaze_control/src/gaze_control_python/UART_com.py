import numpy as np
import serial


#-------- INIT SERIAL PORT -------#
# ser = serial.Serial('/dev/ttyACM0', 115200, parity='N') # Linux use ls /dev/tty*
# ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N') # Mac use ls /dev/tty.*
ser = serial.Serial('COM14', 115200, parity='N') # Windows



#------ DEFINE COMS PROTOCOL ------#
MOVE_LENGTH_REGISTER_ADDRESS = 13
UPDATE_POSITION_REGISTER_ADDRESS = 15
RUN_PROGRAM_REGSITER_ADDRESS = 17
LIGHT_COLOR_REGSITER_ADDRESS = 19

RANDOM_GAZE_PROGRAM = 1
JOINT_POSITION_PROGRAM = 2



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

def send_joint_msg(joint_num, joint_value_rads):
    joint_num = bound_joint_num(joint_num)
    des_enc_cmd = joint_cmd_to_enc_cmd(joint_num, joint_value_rads)

    register = joint_num
    register_value = des_enc_cmd

    msg_to_send = create_binary_msg(register, register_value)

    ser.write(msg_to_send)

    # print ser.readline()
    # print ser.readline()

    return 0

def send_run_program_msg(program_num):
    register = RUN_PROGRAM_REGSITER_ADDRESS
    msg_to_send = create_binary_msg(register, program_num)
    ser.write(msg_to_send)

    # print ser.readline()
    # print ser.readline()
    # print ser.readline()

    return 0

def send_ear_color_msg(color_value):
    register = LIGHT_COLOR_REGSITER_ADDRESS
    msg_to_send = create_binary_msg(register, color_value)
    ser.write(msg_to_send)

    # print ser.readline()
    # print ser.readline()
    # print ser.readline()

    return 0

def send_start_joint_cmd_mvt():
    register = UPDATE_POSITION_REGISTER_ADDRESS
    msg_to_send = create_binary_msg(register, 0)
    ser.write(msg_to_send)

    return 0

def send_move_length_msg(num_ctrl_cycles):  # ctrl loop is 550 Hz => 0.0018s
    register = MOVE_LENGTH_REGISTER_ADDRESS
    msg_to_send = create_binary_msg(register, num_ctrl_cycles)
    ser.write(msg_to_send)

    return 0

#send_joint_msg(3, degs_to_rads(2))
# print ser.readline()
# print ser.readline()
# print ser.readline()


#import rospy
# def callbback
# ros_init
# listens
