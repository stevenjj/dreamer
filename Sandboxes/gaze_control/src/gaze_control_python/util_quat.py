#!/usr/bin/env python
import numpy as np
def Normalize(V):
#Takes a vector.
#Scales it to a unit vector.
    '''
Example Input: 
V = [1, 2, 3]
Output:
[0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
    '''
    return V / np.linalg.norm(V)

def NearZero(z):
#Takes a scalar.
#Checks if the scalar is small enough to be neglected.
    '''
Example Input:
z = -1e-7
Output:
True
    '''
    return abs(z) < 1e-6
   

# Accepts a w_hat \in R^3 (with ||w|| = 1) and a theta \in [0, pi] displacement
# and coverts it to quaternion representation q = [qo, q1, q2, q3, q4] = [cos (theta/2), w_hat*sin(theta/2)]

# Input: w = [1, 0, 0] 
# Input theta = np.pi/2.0
# Output = np.array([sqrt(2)/2, sqrt(2)/2, 0, 0])
def wth_to_quat(w_hat, theta):
	w = np.array(Normalize(w_hat))*(np.sin(theta/2.0))
	return np.array([np.cos(theta/2.0), w[0], w[1], w[2]])


def quat_to_wth(q):
    theta = 2*np.arccos(q[0])
    #print theta, q_error[0], q_error
    if (NearZero(1.0 - q[0])):
        return 0, q[1:]

    factor = np.sin(theta/2.0)
    w_hat_x = q[1]/factor
    w_hat_y = q[2]/factor
    w_hat_z = q[3]/factor        

    angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])
    return theta, angular_vel_hat    


# Accepts a rotation matrix R and converts it to a quaternion
''' Input:
R = [[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]]
Output:
array([ 0.70710678,  0.70710678,  0.        ,  0.        ])
'''
def R_to_quat(R):
	q0 = 0.5 * np.sqrt(1 + R[0][0] + R[1][1] + R[2][2])
	q123 = (1/(4.0*q0)) * np.array( [(R[2][1] - R[1][2]), \
									 (R[0][2] - R[2][0]), \
									 (R[1][0] - R[0][1]) ] ) 
	return np.array([q0, q123[0], q123[1], q123[2]])


# Accepts a unit quaternion
#Input:
'''
q = [ np.sqrt(2)/2.0,  np.sqrt(2)/2.0,  0.        ,  0.        ]
Output:
R = [[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]]
'''
def quat_to_R(q):
	q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
	R = [ [(q0*q0 + q1*q1 - q2*q2 - q3*q3) ,       2*(q1*q2 - q0*q3)         ,           2*(q0*q2 + q1*q3)], \
		  [          2*(q0*q3 + q1*q2)     , (q0*q0 - q1*q1 + q2*q2 - q3*q3) ,           2*(q2*q3 - q0*q1)], \
		  [          2*(q1*q3 - q0*q2)     ,       2*(q0*q1 + q2*q3)         ,    (q0*q0 - q1*q1 - q2*q2 + q3*q3)] ]
	return R

# Accepts unit quaternions q and p and performs the unit quaternion product:
# n = q*p
# Rn = Rq*Rp
# Input:
'''
Rq = np.array([[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]])
Rp = Rq.T
q = R_to_quat(Rq)
p = R_to_quat(Rp)
Output
quat_to_R(n)
R = [[1,0,0],
	[0,1,0],
	[0,0,1]]
'''
def quat_multiply(q, p):
	q = Normalize(q)
	p = Normalize(p)
	q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
	p0, p1, p2, p3 = p[0], p[1], p[2], p[3]	
	n0 = q0*p0 - q1*p1 - q2*p2 - q3*p3
	n1 = q0*p1 + p0*q1 + q2*p3 - q3*p2
	n2 = q0*p2 + p0*q2 - q1*p3 + q3*p1
	n3 = q0*p3 + p0*q3 + q1*p2 - q2*p1
	return np.array([n0, n1, n2, n3])

# Accepts a unit quaternion and returns its inverse / conjugate
'''
Input:
q = [ np.sqrt(2)/2.0,  np.sqrt(2)/2.0,  0.        ,  0.        ]
Output:
[ np.sqrt(2)/2.0,  -np.sqrt(2)/2.0,  0.        ,  0.        ]

'''
def conj(q):
	q = Normalize(q)
	return np.array( [q[0], -q[1], -q[2], -q[3]] )



R = [[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]]
q = [ np.sqrt(2)/2.0,  np.sqrt(2)/2.0,  0.        ,  0.        ]
print R_to_quat(R)
print quat_to_R(q)
Rq = np.array([[1, 0,  0], 
     [0, 0, -1], 
     [0, 1,  0]])
Rp = Rq.T
q = R_to_quat(Rq)
p = R_to_quat(Rp)
print "p = Rp, q = Rp^-1"
print "Quaternion Multiply p*q"
n = quat_multiply(q, p)
print n
print "n = R ="
print quat_to_R(n) #expect Identity matrix

print "quaternion conjugate test. We expect identity matrix:"
print quat_to_R( quat_multiply(q, conj(q)) ) #expect identity matrix
