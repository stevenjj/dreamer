#!/usr/bin/env python

# Math Dependencies
import numpy as np



#J1 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# J1 = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])
# J2 = np.array([[1, 1, 1, 1]])
# J3 = np.array([[1, 1, 0, 0]])


J1 = np.array([[0,0,0,0,0,1,0]])
J2 = np.array([[-1,0,0,-1,0,-1,0],[0,1,0,0,0,1,0],[-1,0,0,-1,-1,0,0],[0,1,0,0,0,0,1]])

#J2 = np.array([[-1,0,0,-1,0,-1,0],[0,1,0,0,0,1,0]])

J3 = np.array([[0,0,-1,0,0,0,0], [-1,0,0,-1,0,0,0],[0,1,0,0,0,0,0]])	           

#dx1 = np.array([0, 0, 0])
dx1 = np.array([[0]])
dx2 = np.array([[0.0],[0],[0],[0]])
dx3 = np.array([[0.0],[0.2],[0.5]])

dq1 = np.linalg.pinv(J1).dot(dx1)

pinvJ1_J1 = np.linalg.pinv(J1).dot(J1)
N1 = np.eye(7) - pinvJ1_J1

dq2 = np.linalg.pinv(J2.dot(N1)).dot(dx2 - J2.dot(dq1))

pinvJ1N1_J1N1 = (np.linalg.pinv(J2.dot(N1))).dot(J2.dot(N1))
N2_1 = np.eye(7) - pinvJ1N1_J1N1

pinvJ3N1N2_1 = np.linalg.pinv( J3.dot(N1.dot(N2_1)).round(decimals=12) )

dq3 = pinvJ3N1N2_1.dot( dx3 - J3.dot(dq1 + dq2) )

#pinvJ2N2_J2N2 = (np.linalg.pinv(J2.dot(N2_1))).dot(J2.dot(N2_1))
#N3_2 = np.eye(np.shape(pinvJ2N2_J2N2)[0]) - pinvJ2N2_J2N2

print 'J1'
print J1
print 'J2'
print J2
print 'J3'
print J3

print 'N1 '
print N1

print 'pinvJ1N1_J1N1'
print pinvJ1N1_J1N1
print 'N2_1 '
print N2_1

print 'np.linalg.pinv(J2.dot(N1))'
print np.linalg.pinv(J2.dot(N1))

print '(J3.dot(N1.dot(N2_1))'
print J3.dot(N1.dot(N2_1))

print 'pinvJ3N1N2_1'
print pinvJ3N1N2_1

print 'np.linalg.matrix_rank((J2.dot(N1)))'
print np.linalg.matrix_rank((J2.dot(N1)))

print 'np.linalg.matrix_rank(N2_1)'
print np.linalg.matrix_rank(N2_1)


print 'N1 N2_1'
print N1.dot(N2_1)

print 'J2.dot(N1)'
print J2.dot(N1)

print 'dq1'
print dq1
print 'dq2'
print dq2

print 'dq3'
print dq3

print 'dq final'
print dq1 + dq2 + dq3

pinvJ2_J2 = np.linalg.pinv(J2).dot(J2)
N2 = np.eye(np.shape(pinvJ2_J2)[0]) - pinvJ2_J2
print 'N2'
print N2