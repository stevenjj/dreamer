# address = format(joint_num, '07b')
# value = format(raw_cmd, '024b')
# bit_cmd = address + value
# parity = sum([int(i) for i in bit if int(i) == 1])/2 % 2
# hex( int(format(1,'04b'),2) )
 
import numpy as np


# ser = serial.Serial('/dev/ttyACM0', 115200, parity='N') # Linux use ls /dev/tty*
# ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N') # Mac use ls /dev/tty.*
# ser = serial.Serial('COM14', 115200, parity='N') Windows


# -------- CONFIG FILE from Microcontroller --------
JOINTS_MAX = {3: 13000}
JOINTS_MIN = {3:  9000}
JOINTS_CEN = {3: JOINTS_MIN[3] + ((JOINTS_MAX[3] - JOINTS_MIN[3])/2)}
 
ENC_MAX = 16384 # (2^14)
ENC_MIN = 0
 
# -------- HELPER FUNCTIONS --------
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
 
 
 
des_joint_num = 3
joint_num = bound_joint_num(des_joint_num)
 
des_degs = 2
des_rads = degs_to_rads(des_degs)
des_enc_cmd = joint_cmd_to_enc_cmd(joint_num, des_rads)
 
raw_cmd = bound_enc_cmd(joint_num, des_enc_cmd)
 
 
#joint_num = 3
#raw_cmd = 12000
 
register = joint_num
register_value = raw_cmd
 
#register = 17
#register_value = 2
 
# ----- MAIN MESSAGING -------#
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
 
 
#----------- DEBUG STATEMENTS ---------------#
print '  Desired (joint, rads, degs, raw_cmd): '
print '         ', (des_joint_num, des_rads, des_degs, des_enc_cmd)
print ''
 
#print '  Encoder Limits:(max enc, min enc, max rads, min rads, max degs, min degs):'
#(max_enc, min_enc, max_rads, min_rads) = (JOINTS_MAX[joint_num], JOINTS_MIN[joint_num], enc_to_rads(JOINTS_MAX[joint_num]), enc_to_rads(JOINTS_MIN[joint_num]) )
#(max_degs, min_degs) = rads_to_deg(max_rads), rads_to_deg(min_rads)
#print '              ', (max_enc, min_enc, max_rads, min_rads, max_degs, min_degs)
#print ''
print '  Encoder Limits:(max enc, min enc):'
(max_enc, min_enc) = (JOINTS_MAX[joint_num], JOINTS_MIN[joint_num])
print '                ', (max_enc, min_enc)
print ''
 
print '  Joint Limits from center: (enc_cen, max_enc_from_cen, min_enc_from_cen)'
print '                            (max_rads_from_cen, min_rads_from_cen)'
print '                            (max_degs_from_cen, min_degs_from_cen)'
enc_cen = JOINTS_CEN[joint_num]
(max_enc_from_cen, min_enc_from_cen) = (max_enc - enc_cen, min_enc - enc_cen)
(max_rads_from_cen, min_rads_from_cen) = (enc_to_rads(max_enc_from_cen), enc_to_rads(min_enc_from_cen))
(max_degs_from_cen, min_degs_from_cen) = (rads_to_deg(max_rads_from_cen), rads_to_deg(min_rads_from_cen))
print ''
print '                            ', (enc_cen, max_enc_from_cen, min_enc_from_cen)
print '                            ', (max_rads_from_cen, min_rads_from_cen)
print '                            ', (max_degs_from_cen, min_degs_from_cen)
print ''
 
print '    Final commands:'
print '    Command Register:', register, 'Value:', register_value, 'Parity:', parity, 'Bit Sum:', bit_sum
print '      In Binary:           ', binary_form
print '         formatted:        ', binary_form[0:7], binary_form[7:31], binary_form[31:32]
print '      In Hex:              ', hex_form
print '      In Base-10 Integer:  ', integer_form
print '      Serial String Command:', msg_to_send
 
 
MOVE_LENGTH_REGISTER_ADDRESS = 7
UPDATE_POSITION_REGISTER_ADDRESS = 15
RUN_PROGRAM_REGSITER_ADDRESS = 17
LIGHT_COLOR_REGSITER_ADDRESS = 19
 
def create_binary_msg(register, register_value):
    return 0
 
def send_joint_msg(joint_num, joint_value):
    return 0
 
def send_run_program_msg():
    return 0
 
def send_ear_color_msg():
    return 0
 
def execute_des_joint():
    register = UPDATE_POSITION_REGISTER
#   create_binary_msg()
    return 0   


#import rospy
# def callbback
# ros_init
# listens