# address = format(joint_num, '07b')
# value = format(raw_cmd, '024b')
# bit_cmd = address + value
# parity = sum([int(i) for i in bit if int(i) == 1])/2 % 2
# hex( int(format(1,'04b'),2) )

from UART_com import *

des_joint_num = 0
joint_num = bound_joint_num(des_joint_num)

des_degs = 0
des_rads = degs_to_rads(des_degs)
des_enc_cmd = joint_cmd_to_enc_cmd(joint_num, des_rads)

# register = joint_num
# register_value = des_enc_cmd




# register = UPDATE_POSITION_REGISTER_ADDRESS
# register = RUN_PROGRAM_REGSITER_ADDRESS
register = MOVE_LENGTH_REGISTER_ADDRESS


# register_value = REMOTE_CONTROL_PROGRAM
# register_value = RANDOM_GAZE_PROGRAM
register_value = 550



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
# msg_to_send = str(integer_form) + "\r" # message to send is the integer value.
msg_to_send = create_binary_msg(register,register_value)



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


print ''
print '    Test Decode:'
print '      ' + decode_message(msg_to_send)