#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "modernRobotics.h"

/* Function: Change an exponential representation to quaternion
 * Inputs: angular velocity vector, rotation about that vector
 * Returns: Quaternion
 */
Eigen::RowVector4d wthToQuat(const Eigen::RowVector3d& wHat, double theta){
	Eigen::RowVector3d w = Normalize(wHat) * std::sin(theta/2);
	Eigen::RowVector4d v_ret(std::cos(theta/2.0), w(0), w(1), w(2));
	return v_ret; 
}

/* Function: Changes a quaternion to the exponential representation
 * Inputs: Quaternion
 * Returns: Exponential representation
 */
Eigen::RowVector3d quatToWth(const Eigen::RowVector4d& q){
	Eigen::RowVector3d v_ret(0,0,0);
	if(NearZero(1 - q(0))){
		return v_ret;
	}

	double theta = 2*std::acos(q(0));
	double factor = std::sin(theta/2);
	v_ret << (q(1)/factor), (q(2)/factor), (q(3)/factor);
	return v_ret * theta;
}

/* Function: Converts a rotation matrix to a quaternion
 * Inputs: Rotation matrix
 * Returns: Quaternion
 */
Eigen::RowVector4d RToQuat(const Eigen::Matrix3d& R){
	double q0 = .5 * std::sqrt(1 + R(0,0) + R(1,1) + R(2,2));
	Eigen::RowVector3d q123( R(2,1) - R(1,2),	 R(0,2) - R(2,0),	 R(1,0) - R(0,1) ); 
	q123 = q123 * (1 / (4 * q0));
	Eigen::RowVector4d v_ret(q0, q123(0), q123(1),q123(2));
	return v_ret;
}

/* Function: Converts a quaternion to a rotation matrix
 * Inputs: Quaternion
 * Returns: Rotation matrix
 */
Eigen::Matrix3d quatToR(const Eigen::RowVector4d& q){
	double q0, q1, q2, q3;
	q0 = q(0);
	q1 = q(1); 
	q2 = q(2);
	q3 = q(3);
	Eigen::Matrix3d m_ret;
	m_ret << (q0*q0 + q1*q1 - q2*q2 - q3*q3) ,       2*(q1*q2 - q0*q3)         ,           2*(q0*q2 + q1*q3),
		          2*(q0*q3 + q1*q2)     , (q0*q0 - q1*q1 + q2*q2 - q3*q3) ,           2*(q2*q3 - q0*q1),
		          2*(q1*q3 - q0*q2)     ,       2*(q0*q1 + q2*q3)         ,    (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    return m_ret;
}

/* Function: Provides the unit quaternion product of two quaternions
 * Inputs: 2 Quaternions
 * Returns: Unit quaternion product
 * Notes: Copy needed
 */
Eigen::RowVector4d quatMultiply(Eigen::RowVector4d q, Eigen::RowVector4d p){
	q = Normalize(q);
	p = Normalize(p);

	double n0, n1, n2, n3;
	n0 = q(0)*p(0) - q(1)*p(1) - q(2)*p(2) - q(3)*p(3);
	n1 = q(0)*p(1) + p(0)*q(1) + q(2)*p(3) - q(3)*p(2);
	n2 = q(0)*p(2) + p(0)*q(2) - q(1)*p(3) + q(3)*p(1);
	n3 = q(0)*p(3) + p(0)*q(3) + q(1)*p(2) - q(2)*p(1);

	Eigen::RowVector4d r_ret(n0, n1, n2, n3);
	return r_ret;
}

/* Function: Returns the conjugate of a unit quaternion
 * Inputs: Quaternion
 * Returns: Inverse/conjugate of the quaternion
 * Notes: Copy needed
 */
Eigen::RowVector4d conj(Eigen::RowVector4d q){
	q = Normalize(q);
	q << q(0), -q(1), -q(2), -q(3);
	return q;
}



/*
Eigen::Matrix3d R;
	R << 1,0,0,
		0,0,-1,
		0,1,0;
	Eigen::RowVector4d q(std::sqrt(2)/2.0, std::sqrt(2)/2.0, 0, 0 );

	std::cout << RToQuat(R) << std::endl;
	std::cout << quatToR(q) << std::endl;
	
	Eigen::Matrix3d Rq;
	Rq << 1,0,0,
		0,0,-1,
		0,1,0;
	Eigen::Matrix3d Rp = Rq.transpose();

	std::cout << quatToR(quatMultiply(RToQuat(Rq), RToQuat(Rp))) << std::endl;

	std::cout << quatToR(quatMultiply(q, conj(q))) << std::endl;
*/
