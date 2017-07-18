#include <iostream>
#include <vector>
// Must add Eigen to /usr/local/include
#include <Eigen/Dense>
#include <ctime>
#include <cmath>


bool NearZero(double val){
	return (std::abs(val) < .000001);
}

Eigen::MatrixXd Normalize(Eigen::MatrixXd V){
	V.normalize();
	return V;
}

/* Function: Returns the skew symmetric matrix of an angular velocity vector
 * Input: Eigen::Vecotor3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d VecToso3(Eigen::Vector3d omg) {
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
Eigen::Vector3d so3ToVec(Eigen::MatrixXd so3mat) {
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
Eigen::Vector4d AxisAng3(Eigen::Vector3d expc3){
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
Eigen::Matrix3d MatrixExp3(Eigen::Matrix3d so3mat){
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


Eigen::MatrixXd* TransToRp(Eigen::MatrixXd T){
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
Eigen::MatrixXd VecTose3(Eigen::VectorXd V){
	Eigen::Vector3d temp;
	temp << V(0), V(1), V(2);

	Eigen::Vector3d temp2;
	temp2 << V(3), V(4), V(5);

	Eigen::MatrixXd m_ret(4,4);
	m_ret << VecToso3(temp), temp2,
			0, 0, 0, 0;

	return m_ret;
}

// Memory leaks suck
Eigen::MatrixXd Adjoint(Eigen::MatrixXd T){
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
Eigen::MatrixXd MatrixExp6(Eigen::MatrixXd se3mat){
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

/* Function: 
 * 
 */
Eigen::MatrixXd JacobianSpace(Eigen::MatrixXd Slist,Eigen::MatrixXd thetaList) {
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


int main()
{
	// // Testing AxisAng3
	// Eigen::Vector3d v3;
	// v3 << 1,2,3;
	// std::cout<< AxisAng3(v3) << std::endl;
	

	// // Testing Matrix3d
	// Eigen::Matrix3d m3;
	// m3 << 0,-3,2,
	// 	3,0,-1,
	// 	-2,1,0;
	// MatrixExp3(m3);

	// // Testing MatrixExp6
	// Eigen::MatrixXd mx(4,4);
	// mx <<0,                 0,                  0,                 0,
	//         0,                 0, -1.570796326794897, 2.356194490192345,
	//         0, 1.570796326794897,                  0, 2.356194490192345,
	//         0,                 0,                  0,                 0;
	//    std::cout << MatrixExp6(mx) << std::endl;

	Eigen::MatrixXd sList(4,6);
	sList << 0, 0, 1,   0, 0.2, 0.2, 
                  1, 0, 0,   2,   0,   3, 
                  0, 1, 0,   0,   2,   1, 
                  1, 0, 0, 0.2, 0.3, 0.4;

    Eigen::Vector4d thetaList;
    thetaList << .2, 1.1, .1, 1.2;
    std::cout << JacobianSpace(sList.transpose(), thetaList) << std::endl;


 //    Eigen::MatrixXd adTest(4,4);
 //    adTest << 1, 0,  0, 0, 
	//      0, 0, -1, 0, 
	//      0, 1,  0, 3, 
	//      0, 0,  0, 1;
	// std::cout<<Adjoint(adTest)<<std::endl;

	//	//transtorp test
	//    Eigen::MatrixXd transTest(4,4);
	//    adTest << 1, 0,  0, 0, 
	//      0, 0, -1, 0, 
	//      0, 1,  0, 3, 
	//      0, 0,  0, 1;
	// Eigen::MatrixXd *test;
	// test = TransToRp(adTest);
	// std::cout << test[1] << std::endl;






	// std::cout<< MatrixExp3(m3) << std::endl;

	// std::clock_t start;
	// start = std::clock();
	// Eigen::Vector3d b(1, 1, 0);
	// b.normalize();
	// std::cout << b << std::endl;
	// std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC) << " s" << std::endl;
	// return 0;
}