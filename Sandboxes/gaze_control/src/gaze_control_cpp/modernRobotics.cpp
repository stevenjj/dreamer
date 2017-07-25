// Must add Eigen to /usr/local/include
#include <Eigen/Dense>
#include <cmath>

/*TODO:
	- add const stuff?
	- pass references if I don't mess with things
	- Don't use eigen dense if not needed
*/

/* Function: Find if the value is negligible enough to consider 0
 * Input: value to be checked as a double
 * Output: Boolean of true-ignore or false-can't ignore
 */
bool NearZero(const double val){
	return (std::abs(val) < .000001);
}

/* Function: Returns a normalized version of the input vector
 * Input: Eigen::MatrixXd
 * Output: Eigen::MatrixXd
 * Note: MatrixXd is used instead of VectorXd for the case of row vectors 
 * 		Requires a copy
 */
Eigen::MatrixXd Normalize(Eigen::MatrixXd V){
	V.normalize();
	return V;
}

/* Function: Returns the skew symmetric matrix of an angular velocity vector
 * Input: Eigen::Vecotor3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
	Eigen::Matrix3d m_ret;
	m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
	return m_ret;
}


/* Function: Returns angular velocity vector from skew symmetric matrix
 * Inputs: Eigen::MatrixXd 3x3 skew symmetric matrix
 * Returns: Eigen::Vector3d 3x1 angular velocity
 */
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
	Eigen::Vector3d v_ret;
	v_ret << so3mat(2,1), so3mat(0,2), so3mat(1,0);
	return v_ret;
}

/*
#Takes A 3-vector of exponential coordinates for rotation.
#Returns unit rotation axis omghat and the corresponding rotation angle
#theta.
    '''
Example Input: 
expc3 = [1, 2, 3]
Output:
([0.2672612419124244, 0.5345224838248488, 0.8017837257372732],
 3.7416573867739413) 
    '''
    */
Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3){
	Eigen::Vector4d v_ret;
	v_ret << Normalize(expc3), expc3.norm();
	return v_ret;
}

// #Takes a so(3) representation of exponential coordinates.
// #Returns R in SO(3) that is achieved by rotating about omghat by theta from
// #an initial orientation R = I.
//     '''
// Example Input: 
// so3mat = [[ 0, -3,  2],
// 	  [ 3,  0, -1],
//           [-2,  1,  0]]
// Output:
// [[-0.69492056,  0.71352099,  0.08929286],
//  [-0.19200697, -0.30378504,  0.93319235],
//  [ 0.69297817,  0.6313497 ,  0.34810748]]
//     '''
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat){
	Eigen::Vector3d omgtheta = so3ToVec(so3mat);

	Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
	if( NearZero(so3mat.norm()) ) {
		return m_ret;
	}
	else {
		double theta = (AxisAng3(omgtheta))(3);
		Eigen::Matrix3d omgmat = so3mat * (1/theta);
		return m_ret + std::sin(theta) * omgmat + ( (1 - std::cos(theta)) * (omgmat * omgmat));
	}
}

Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p){
	Eigen::MatrixXd m_ret(4,4);
	m_ret << R, p,
		0, 0, 0, 1;
	return m_ret;
}


Eigen::MatrixXd* TransToRp(const Eigen::MatrixXd& T){
	static Eigen::MatrixXd Rp_ret[2];
	Eigen::Matrix3d R_ret;
	R_ret = T.block<3,3>(0,0);

	Eigen::Vector3d p_ret;
    p_ret << T(0,3), T(1,3), T(2,3);
    
    Rp_ret[0] = R_ret;
    Rp_ret[1] = p_ret;

    return Rp_ret;
}


/* Function: Returns an se3 matrix from a spatial velocity vector
 *
 */
Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V){
	Eigen::Vector3d temp;
	temp << V(0), V(1), V(2);

	Eigen::Vector3d temp2;
	temp2 << V(3), V(4), V(5);

	Eigen::MatrixXd m_ret(4,4);
	m_ret << VecToso3(temp), temp2,
			0, 0, 0, 0;

	return m_ret;
}

/*#Takes T a transformation matrix SE(3).
#Returns the corresponding 6x6 adjoint representation [AdT].
    '''
Example Input: 
T = [[1, 0,  0, 0], 
     [0, 0, -1, 0], 
     [0, 1,  0, 3], 
     [0, 0,  0, 1]]
Output:
[[1, 0,  0, 0, 0,  0],
 [0, 0, -1, 0, 0,  0],
 [0, 1,  0, 0, 0,  0],
 [0, 0,  3, 1, 0,  0],
 [3, 0,  0, 0, 0, -1],
 [0, 0,  0, 0, 1,  0]]
    '''*/
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T){
	Eigen::MatrixXd *R = TransToRp(T);
	Eigen::MatrixXd ad_ret(6,6);
	Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3,3);
	ad_ret << R[0], zeroes,
			VecToso3(R[1]) * R[0], R[0];
	return ad_ret;
}



/*
#Takes a se(3) representation of exponential coordinates.
#Returns a T matrix SE(3) that is achieved by traveling along/about the
#screw axis S for a distance theta from an initial configuration T = I.
    '''
Example Input: 
se3mat = [[0,                 0,                  0,                 0],
          [0,                 0, -1.570796326794897, 2.356194490192345],
          [0, 1.570796326794897,                  0, 2.356194490192345],
          [0,                 0,                  0,                 0]]
Output:
[[1.0, 0.0,  0.0, 0.0],
 [0.0, 0.0, -1.0, 0.0],
 [0.0, 1.0,  0.0, 3.0],
 [  0,   0,    0,   1]]
    '''  */
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat){
	Eigen::Matrix3d se3mat_cut = se3mat.block<3,3>(0,0);
	Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);
	Eigen::MatrixXd m_ret(4,4);

	if(NearZero(omgtheta.norm())){
		// Reuse previous variables that have our needed size
		se3mat_cut = Eigen::MatrixXd::Identity(3,3);
		omgtheta << se3mat(0,3), se3mat(1,3), se3mat(2,3);
		m_ret << se3mat_cut, omgtheta,
			0, 0, 0, 1;
		return m_ret;
	}
	else{
		double theta = (AxisAng3(omgtheta))(3);
		Eigen::Matrix3d omgmat = se3mat.block<3,3>(0,0) / theta;
		Eigen::Matrix3d temp = Eigen::MatrixXd::Identity(3,3) * theta + (1-std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
		Eigen::Vector3d temp2(se3mat(0,3), se3mat(1,3), se3mat(2,3));
		temp2 = (temp*temp2) / theta;
		m_ret << MatrixExp3(se3mat_cut), temp2,
				0, 0, 0, 1;
		return m_ret;
	}


}


/*#Takes M: the home configuration (position and orientation) of the 
#         end-effector,
#      Slist: The joint screw axes in the space frame when the manipulator
#             is at the home position,
#      thetalist: A list of joint coordinates.
#Returns T in SE(3) representing the end-effector frame when the joints are
#at the specified coordinates (i.t.o Space Frame).
    '''
Example Input: 
import numpy as np
from math import pi
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T
thetalist = [pi / 2.0, 3, pi]
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
 [  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
 [              0.               0.              -1.       1.68584073],
 [              0.               0.               0.               1.]]
    '''
 * Note: Requires M be a copy
 */
Eigen::MatrixXd FKinSpace(Eigen::MatrixXd M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){
	for(int i=(thetaList.size()-1); i>-1; i--){
			M = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * M;
	}
	return M;
}	

/*
#Takes Slist: The joint screw axes in the space frame when the manipulator
#             is at the home position,
#      thetalist: A list of joint coordinates.
#Returns the corresponding space Jacobian (6xn real numbers).
    '''
Example Input: 
import numpy as np
Slist = np.array([[0, 0, 1,   0, 0.2, 0.2], 
                  [1, 0, 0,   2,   0,   3], 
                  [0, 1, 0,   0,   2,   1], 
                  [1, 0, 0, 0.2, 0.3, 0.4]]).T
thetalist = [0.2, 1.1, 0.1, 1.2]
Output:
[[ 0.          0.98006658 -0.09011564  0.95749426]
 [ 0.          0.19866933  0.4445544   0.28487557]
 [ 1.          0.          0.89120736 -0.04528405]
 [ 0.          1.95218638 -2.21635216 -0.51161537]
 [ 0.2         0.43654132 -2.43712573  2.77535713]
 [ 0.2         2.96026613  3.23573065  2.22512443]]
    '''   
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList) {
	Eigen::MatrixXd Js = Slist;
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
	Eigen::VectorXd sListTemp(Slist.col(0).size());
	for(int i = 1; i < thetaList.size(); i++) {
		sListTemp << Slist.col(i-1) * thetaList(i-1);
		T = T * MatrixExp6(VecTose3(sListTemp));
		// std::cout << "array: " << sListTemp << std::endl;
		Js.col(i) = Adjoint(T) * Slist.col(i);
	}

	return Js;
}
