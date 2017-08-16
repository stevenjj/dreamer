#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "modernRobotics.h"
#include "headKinematics.h"

headKinematics::headKinematics(void){
	// Dreamer Joint links
	l0 = 0.3115;
	l1 = 0.13849;
	l2 = 0.12508;
	l3 = 0.053;
	
	// Screw axis for each joint
	S0 = Eigen::VectorXd::Zero(6);
	S0 << 0, -1, 0, 0, 0, 0; 
	S1 = Eigen::VectorXd::Zero(6);
	S1 << 0, 0, 1, 0, 0, 0;
	S2 = Eigen::VectorXd::Zero(6);
	S2 << -1, 0, 0, 0, -l1, 0;  
	S3 = Eigen::VectorXd::Zero(6);
	S3 << 0, -1, 0, l1, 0, 0;  
	S4 = Eigen::VectorXd::Zero(6);
	S4 << 0, -1, 0, l1, 0, -l2;  
	S5 = Eigen::VectorXd::Zero(6);
	S5 << 0, 0, 1, -l3, -l2, 0;  
	S6 = Eigen::VectorXd::Zero(6);
	S6 << 0, 0, 1, l3, -l2, 0;  

	rHeadHome = Eigen::MatrixXd::Identity(3,3);
	rRightEyeHome = Eigen::MatrixXd::Identity(3,3);
	rLeftEyeHome = Eigen::MatrixXd::Identity(3,3);


	pHeadHome << 0, 0, l1;
	pRightEyeHome << l2, -l3, l1;
	pLeftEyeHome << l2, l3, l1;
	
	screwAxisTables = Eigen::MatrixXd::Zero(7,6);
	screwAxisTables << S0, S1, S2, S3, S4, S5, S6;

	J_num = screwAxisTables.rows();

	Jlist = Eigen::VectorXd::Zero(screwAxisTables.rows());

	JIndexToNames[0] = "lower_neck_pitch";
	JIndexToNames[1] = "upper_neck_yaw";
	JIndexToNames[2] = "upper_neck_roll";
	JIndexToNames[3] = "upper_neck_pitch";
	JIndexToNames[4] = "eye_pitch";
	JIndexToNames[5] = "right_eye_yaw";
	JIndexToNames[6] = "left_eye_yaw";
}

headKinematics::~headKinematics(void){}


/* Function: Gives the 6D Jacobian for Dreamer's head
 * Inputs: Joint configuration
 * Returns: 6D Spatial Head Jacobian
 */
Eigen::MatrixXd headKinematics::get6D_HeadJacobian(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 3;
	const int numJoints = screwAxisEnd + 1;
	Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
	Eigen::RowVectorXd thetaList = JList.head(numJoints); // First 4 joints
	Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


	Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
	Eigen::MatrixXd JSpatial_6DHead(6,7);

	JSpatial_6DHead << JSpatial, zeroVec;

	return JSpatial_6DHead;
}


/* Function: Gives the 6D Jacobian for Dreamer's Right Eye
 * Inputs: Joint configuration
 * Returns: 6D Spatial Right Eye Jacobian
 */
Eigen::MatrixXd headKinematics::get6D_RightEyeJacobian(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 5;
	const int numJoints = screwAxisEnd + 1;
	Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
	Eigen::RowVectorXd thetaList = JList.head(numJoints); // First 6 joints
	Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


	Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
	Eigen::MatrixXd JSpatial_6DRightEye(6,7);

	JSpatial_6DRightEye << JSpatial, zeroVec;

	return JSpatial_6DRightEye;
}


/* Function: Gives the 6D Jacobian for Dreamer's Left Eye
 * Inputs: Joint configuration
 * Returns: 6D Spatial Left Eye Jacobian
 */
Eigen::MatrixXd headKinematics::get6D_LeftEyeJacobian(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 6;
	const int numJoints = screwAxisEnd;
	Eigen::MatrixXd Slist(6,6);
	Slist << screwAxisTables.block<5, 6>(0,0), screwAxisTables.block<1, 6>(6,0);

	Eigen::RowVectorXd thetaList(6);
	thetaList << JList.head(numJoints-1).transpose(), JList(numJoints);
	
	Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);

	Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
	Eigen::MatrixXd JSpatial_6DLeftEye(6,7);

	// First 5 columns, 0 column, last column
	JSpatial_6DLeftEye << JSpatial.block<6,5>(0,0), zeroVec, JSpatial.block<6,1>(0,5);

	return JSpatial_6DLeftEye;
}


/* Function: Gives the 6D Jacobian for Dreamer's Right Eye exlcuding head joints
 * Inputs: Joint configuration
 * Returns: 6D Spatial Jacobian
 */
Eigen::MatrixXd headKinematics::get6D_RightEyeJacobianYawPitch(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 5;
	const int numJoints = screwAxisEnd + 1;
	Eigen::MatrixXd Slist = screwAxisTables.block<2, 6>(4,0);
	Eigen::RowVectorXd thetaList = JList.segment(4,2); // 2 joint starting from joint 4
	Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


	Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, 4);
	Eigen::MatrixXd JSpatial_6DRightEye(6,7);

	JSpatial_6DRightEye << zeroVec, JSpatial, zeroVec.block<6,1>(0,0);

	return JSpatial_6DRightEye;
}


/* Function: Gives the 6D Jacobian for Dreamer's Left Eye exlcuding head joints
 * Inputs: Joint configuration
 * Returns: 6D Spatial Jacobian
 */
Eigen::MatrixXd headKinematics::get6D_LeftEyeJacobianYawPitch(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 6;
	const int numJoints = screwAxisEnd;
	Eigen::MatrixXd Slist(2,6);
	Slist << screwAxisTables.block<1, 6>(4,0), screwAxisTables.block<1, 6>(6,0);
	Eigen::RowVector2d thetaList(JList(4), JList(6)); 
	Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);
	std::cout << JSpatial << std::endl;

	Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, 4);
	Eigen::MatrixXd JSpatial_6DLeftEye(6,7);

	// First 4 columns of zeros, 1 column, zero column, last column
	JSpatial_6DLeftEye << zeroVec, JSpatial.block<6,1>(0,0) , zeroVec.block<6,1>(0,0),  JSpatial.block<6,1>(0,1);

	return JSpatial_6DLeftEye;
}


/* Function: Gives the spatial position of Dreamer's Head
 * Inputs: Joint configuration
 * Returns: Spatial position of the head
 */
std::vector<Eigen::MatrixXd> headKinematics::get6D_HeadPosition(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 3;
	const int numJoints = screwAxisEnd + 1;
	Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
	Eigen::RowVectorXd thetaList = JList.head(numJoints); // First 4 joints

	Eigen::MatrixXd M_HeadHome = RpToTrans(rHeadHome, pHeadHome);
	
	Eigen::MatrixXd T_Head = FKinSpace(M_HeadHome, Slist.transpose(), thetaList);

	return TransToRp(T_Head);
}


/* Function: Gives the spatial position of Dreamer's Right Eye
 * Inputs: Joint configuration
 * Returns: Spatial position of the right eye
 */
std::vector<Eigen::MatrixXd> headKinematics::get6D_RightEyePosition(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 5;
	const int numJoints = screwAxisEnd + 1;
	Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
	Eigen::RowVectorXd thetaList = JList.head(numJoints); // First 4 joints

	Eigen::MatrixXd M_RightEyeHome = RpToTrans(rRightEyeHome, pRightEyeHome);
	
	Eigen::MatrixXd T_RightEye = FKinSpace(M_RightEyeHome, Slist.transpose(), thetaList);

	return TransToRp(T_RightEye);
}


/* Function: Gives the spatial position of Dreamer's Left Eye
 * Inputs: Joint configuration
 * Returns: Spatial position of the left eye
 */
std::vector<Eigen::MatrixXd> headKinematics::get6D_LeftEyePosition(const Eigen::VectorXd& JList){
	const int screwAxisEnd = 6;
	const int numJoints = screwAxisEnd;

	Eigen::MatrixXd Slist(6,6);
	Slist << screwAxisTables.block<5, 6>(0,0), screwAxisTables.block<1, 6>(6,0);

	Eigen::RowVectorXd thetaList(6);
	thetaList << JList.head(numJoints-1).transpose(), JList(numJoints);

	Eigen::MatrixXd M_LeftEyeHome = RpToTrans(rLeftEyeHome, pLeftEyeHome);
	
	Eigen::MatrixXd T_LeftEye = FKinSpace(M_LeftEyeHome, Slist.transpose(), thetaList);

	return TransToRp(T_LeftEye);
}

