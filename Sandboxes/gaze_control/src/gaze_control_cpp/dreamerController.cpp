#include <Eigen/Dense>
#include <iostream>
#include <ctime>
#include <cstring>
#include <stdexcept>
#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "modernRobotics.h"
#include "headKinematics.h"
#include "utilQuat.h"
#include "dreamerController.h"

const double pi = M_PI;
const double FOCUS_THRESH = 0.08;
const double INIT_FOCUS_LENGTH = 0.6;


/* Function: Rotation operator around a vector
 *           Used for head tilts, Taken from Modern Robotics book
 * Inputs: vector to rotate about, angle to rotate
 * Returns: 3x3 rotation matrix
 */
Eigen::Matrix3d Rot(const Eigen::Vector3d& w_hat, const double theta){
	double w1 = w_hat(0);
	double w2 = w_hat(1);
	double w3 = w_hat(2);
	Eigen::Matrix3d m_ret;
	m_ret <<	std::cos(theta) + (w1*w1)*(1-std::cos(theta)),   w1*w2*(1-std::cos(theta)) - w3*std::sin(theta), w1*w3*(1-std::cos(theta))+w2*std::sin(theta),
                w1*w2*(1-std::cos(theta)) + w3*std::sin(theta),  std::cos(theta) + (w2*w2)*(1-std::cos(theta)),  w2*w3*(1-std::cos(theta))-w1*std::sin(theta),
                w1*w3*(1-std::cos(theta)) - w2*std::sin(theta),  w2*w3*(1-std::cos(theta)) + w1*std::sin(theta), std::cos(theta) + (w3*w3)*(1-std::cos(theta));

    return m_ret;
}


/* Function: Find the change in joint configuration
 * Input: Jacobian matrix for joint, change in operational space
 * Return: Joint change
 * Note: Uses singular value decomposition for a pseudoinverse
 */
Eigen::MatrixXd calculate_dQ(const Eigen::MatrixXd& J, const Eigen::MatrixXd& dx){
	return J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dx);
}



dreamerController::dreamerController(void){
	H = "head";
	RE = "right_eye";
	LE = "left_eye";

	movement_complete = false;

	focusPointInit[H] << 1, 0, 0;
	focusPointInit[RE] << 1, 0, 0;
	focusPointInit[LE] << 1, 0, 0;

	currentFocusLength[H] = INIT_FOCUS_LENGTH;
	currentFocusLength[RE] = INIT_FOCUS_LENGTH;
	currentFocusLength[LE] = INIT_FOCUS_LENGTH;


	// Gaze Focus States
	Eigen::RowVector3d zero = Eigen::VectorXd::Zero(3);

	gazePosition[H] = zero;
	gazePosition[RE] = zero;
	gazePosition[LE] = zero;

	gazeOldPosition[H] = zero;
	gazeOldPosition[RE] = zero;
	gazeOldPosition[LE] = zero;

	gazeVelocity[H] = zero;
	gazeVelocity[RE] = zero;
	gazeVelocity[LE] = zero;

	updateGazeFocus(1.0);

	gazePublisher = n.advertise<std_msgs::Float32MultiArray>("setArrLength", 1);

}

dreamerController::~dreamerController(void){}


/**
 * Function: Updates gaze variables
 * Inputs: time taken from last update (used for velocity calculations)
 * Returns: None
 */
void dreamerController::updateGazeFocus(const double dt = 1.0){
	gazeOldPosition[H] = gazePosition[H];
	gazeOldPosition[RE] = gazePosition[RE];
	gazeOldPosition[LE] = gazePosition[LE];

	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);

	gazePosition[H] = hPoint[0].block<1,3>(0,0) * currentFocusLength[H] + hPoint[1].transpose();
	gazePosition[RE] = rePoint[0].block<1,3>(0,0) * currentFocusLength[RE] + rePoint[1].transpose();
	gazePosition[LE] = lePoint[0].block<1,3>(0,0) * currentFocusLength[LE] + lePoint[1].transpose();


	gazeVelocity[H] = (gazePosition[H] - gazeOldPosition[H])/dt;
	gazeVelocity[RE] = (gazePosition[RE] - gazeOldPosition[RE])/dt;
	gazeVelocity[LE] = (gazePosition[LE] - gazeOldPosition[LE])/dt;
}


/**
 * Function: Published focus length to RVIZ
 * Inputs: None
 * Returns: None
 */
void dreamerController::publishFocusLength(void){
	std_msgs::Float32MultiArray msg;
	msg.data.push_back(currentFocusLength[LE]);
	msg.data.push_back(currentFocusLength[RE]);
	msg.data.push_back(currentFocusLength[H]);
	gazePublisher.publish(msg);

}


/**
 * Function: Helper function to focus eyes
 * Inputs: None - uses class variables
 * Returns: Boolean of True(focused) or False(not focused)
 */
bool dreamerController::gazeAreEyesFocused(void){
	return ( (gazePosition[RE] - gazePosition[LE]).norm() < FOCUS_THRESH );
}


/**
 * Function: Provides a minimum jerk scaling from start time to end time
 * Inputs: current time, total time of motion
 * Returns: min_jerk scaling from 0.0-1.0 based on minimum jerk curve
 */
double dreamerController::minJerkTimeScaling(const double t, const double dt){
	if(t < 0)
		return 0.0;
	else if(t >= dt)
		return 1.0;
	else
		return 10*std::pow(t/dt, 3) - 15*std::pow(t/dt, 4) + 6*std::pow(t/dt, 5); 
}


/**
 * Function: Sets initial parameters for gaze focus length
 * Inputs: operational space head point, operational space eye point
 * Returns: None
 */
void dreamerController::initializeHeadEyeFocusPoint(const Eigen::Vector3d& xyzHead, const Eigen::Vector3d& xyzEye){
	std::vector<Eigen::MatrixXd> head = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> re = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> le = kinematics.get6D_LeftEyePosition(kinematics.Jlist);

	Eigen::Vector3d xhead = head[0].block<3,1>(0, 0);
	Eigen::Vector3d xre = re[0].block<3,1>(0, 0);
	Eigen::Vector3d xle = le[0].block<3,1>(0, 0);
	// If eyes not are focused enough
	if(!gazeAreEyesFocused()){
		currentFocusLength[H] = (xyzHead - head[1]).norm();
		currentFocusLength[RE] = (xyzEye - re[1]).norm();
		currentFocusLength[LE] = (xyzEye - le[1]).norm();
	}

	focusPointInit[H] = (head[1] + xhead * currentFocusLength[H]).transpose();
	focusPointInit[RE] = (re[1] + xre * currentFocusLength[RE]).transpose();
	focusPointInit[LE] = (le[1] + xle * currentFocusLength[LE]).transpose();
}





/* Function: Calculate a rotation matrix based on the desired parameters
 * Inputs: desired gaze location, current spatial position, desired joint configuration, initial join configuration, min_jerk scaling, head component, head roll
 * Returns: Desired Rotation Matrix
 */
Eigen::Matrix3d dreamerController::calcSmoothDesiredOrientation(const Eigen::Vector3d& xGazeLoc, const Eigen::Vector3d& pCur, const Eigen::VectorXd& Q_cur, const Eigen::VectorXd& Q_init, const double scaling, const std::string orientationType = "head", const double tilt = 0) {
	// Find unit vector of the difference between desired position and current position
	Eigen::Vector3d pBar = xGazeLoc - pCur;
	Eigen::Vector3d xHatD = Normalize(pBar);
	// std::cout << pCur << std::endl;

	// Get Rotation and position of the component
	std::vector<Eigen::MatrixXd> pos;
	if(orientationType == "head")
		pos = kinematics.get6D_HeadPosition(Q_init);
	else if(orientationType == "right_eye")
		pos = kinematics.get6D_RightEyePosition(Q_init);
	else if(orientationType == "left_eye")
		pos = kinematics.get6D_LeftEyePosition(Q_init);
	else
		throw std::invalid_argument("Unknown position and orientation needed");

	Eigen::Vector3d xWorldHat(1, 0, 0);
	Eigen::Vector3d yWorldHat(0, 1, 0);
	Eigen::Vector3d zWorldHat(0, 0, 1);

	// phi is the angle between the error vector and the z axis of the world
	// Used to make a projection onto the world z axis/y axis for orientation
	double phi = std::acos( (xHatD.dot(zWorldHat)) / (xHatD.norm()*zWorldHat.norm()));
	double epsilon = .1 * pi/180.0;

	// If the angle previously calculated is too small, we want to project onto the y axis
	Eigen::Vector3d zHatO = zWorldHat;
	if(phi <= epsilon)
		zHatO = yWorldHat;

	// Rotate the world z axis to get the desired head roll
	double theta = tilt;
	Eigen::Matrix3d headTilt = Rot(xWorldHat, theta);
	zHatO = headTilt * zHatO;

	// Get z axis of orientation
	Eigen::Vector3d zHatInit = pos[0].block<3,1>(0,2);

	// Move towards desired z orientation along min_jerk scaling
	zHatO = Normalize(zHatInit + (zHatO - zHatInit) * scaling);

	Eigen::Vector3d zHatD = Normalize(zHatO - (zHatO.dot(xHatD) * xHatD));
	Eigen::Vector3d yHatD = zHatD.cross(xHatD);

	// Assemble the desired rotation matrix
	Eigen::Matrix3d RDesired;
	RDesired << xHatD, yHatD, zHatD;

	return RDesired; 
}


/* Function: Calculates error between current and desired configurations
 * Inputs: desired gaze location, desired joint configuration, current joint configuration, min_jerk time scaling
 * Returns: Error in angle * angular velocities as an array
 */
Eigen::Vector3d dreamerController::smoothOrientationError(const Eigen::Vector3d& xGazeLoc, const Eigen::VectorXd& Q, const Eigen::VectorXd& Q_init, const double scaling, const std::string orientationType = "head", const double tilt = 0){
	std::vector<Eigen::MatrixXd> pos;
	if(orientationType == "head")
		pos = kinematics.get6D_HeadPosition(Q);
	else if(orientationType == "right_eye")
		pos = kinematics.get6D_RightEyePosition(Q);
	else if(orientationType == "left_eye")
		pos = kinematics.get6D_LeftEyePosition(Q);
	else
		throw std::invalid_argument("Unknown position and orientation needed");

	Eigen::Matrix3d R_des = calcSmoothDesiredOrientation(xGazeLoc, pos[1], Q, Q_init, scaling, orientationType, tilt);

	Eigen::RowVector4d qCur = RToQuat(pos[0]);
	Eigen::RowVector4d qDes = RToQuat(R_des);

	Eigen::RowVector4d qError = quatMultiply(qDes, conj(qCur));

	return quatToWth(qError);
}



/**
 * Function: Calculate desired joint positions for go to point with head priority
 * Inputs: xyz head gaze, xyz eye gaze, joint configuration, motion start time, total motion run time
 * Outputs: Desired joint configuation
 */
Eigen::VectorXd dreamerController::headPriorityEyeTrajectoryLookAtPoint(const Eigen::Vector3d& xyzHeadGaze, const Eigen::Vector3d& xyzEyeGaze, const Eigen::VectorXd& initQ, const double sTime, const double tTime){
	double currentTrajectoryTime = ros::Time::now().toSec();
	currentTrajectoryTime -= sTime;
	// double currentTrajectoryTime = 3;
	/************************** Head Calculations **************************/
	// Get only the joints that affect the head orientation
	Eigen::VectorXd Q_cur = kinematics.Jlist;
	Eigen::MatrixXd J_head = kinematics.get6D_HeadJacobian(Q_cur);
	Eigen::MatrixXd J_head_block = J_head.block(0, 0, 3, J_head.cols());

	// Find the head focus length
	Eigen::RowVector3d headFocus = focusPointInit[H];

	// Draw a line between the desired and final vectors
	Eigen::Vector3d error = Normalize(xyzHeadGaze - headFocus.transpose());
	double lHead = (xyzHeadGaze - headFocus.transpose()).norm();
	
	// If the difference between the two vectors is small, do nothing
	if(NearZero(lHead))
		error = Normalize(headFocus).transpose();
		// std::cout << Normalize(headFocus) << std::endl;
	
	// Move towards the desired point along a minimum jerk
	Eigen::Vector3d pHeadDesired = headFocus.transpose() + error * lHead * minJerkTimeScaling(currentTrajectoryTime, tTime);

	// Find the operational space change
	Eigen::Vector3d dxHead = smoothOrientationError(pHeadDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime));
	// std::cout << dxHead << std::endl;


	/************************** Eyes Calculations **************************/
	Eigen::MatrixXd J1 = kinematics.get6D_RightEyeJacobian(Q_cur);
	Eigen::MatrixXd J2 = kinematics.get6D_LeftEyeJacobian(Q_cur);

	Eigen::MatrixXd J1_block = J1.block(0, 0, 3, J1.cols());
	Eigen::MatrixXd J2_block = J2.block(0, 0, 3, J2.cols());

	Eigen::MatrixXd J_eyes(J1_block.rows()+J2_block.rows(), J1_block.cols());
	J_eyes << J1_block,
			  J2_block;

	Eigen::RowVector3d reFocus = focusPointInit[RE];
	Eigen::RowVector3d leFocus = focusPointInit[LE];

	Eigen::Vector3d errorRE = Normalize(xyzEyeGaze - reFocus.transpose());
	Eigen::Vector3d errorLE = Normalize(xyzEyeGaze - leFocus.transpose());
	double lRE = (xyzEyeGaze - reFocus.transpose()).norm();
	double lLE = (xyzEyeGaze - leFocus.transpose()).norm();

	if(NearZero(lRE))
		errorRE = Normalize(reFocus).transpose();
	if(NearZero(lLE))
		errorLE = Normalize(leFocus).transpose();

	Eigen::Vector3d pRightEyeDesired = reFocus.transpose() + errorRE * lRE * minJerkTimeScaling(currentTrajectoryTime, tTime);
	Eigen::Vector3d pLeftEyeDesired = leFocus.transpose() + errorLE * lLE * minJerkTimeScaling(currentTrajectoryTime, tTime);


	Eigen::Vector3d dxRE = smoothOrientationError(pRightEyeDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime), "right_eye");
	Eigen::Vector3d dxLE = smoothOrientationError(pLeftEyeDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime), "left_eye");
	Eigen::MatrixXd dxEyes(dxRE.rows() + dxLE.rows(), dxRE.cols());

	dxEyes << dxRE,
			  dxLE;


	// Specify priority
	J1 = J_head_block;
	Eigen::MatrixXd dx1 = dxHead;
	J2 = J_eyes;
	Eigen::MatrixXd dx2 = dxEyes;
	


	// Calculate joint movement
	Eigen::VectorXd dq1 = calculate_dQ(J1, dx1);
	

	// Calculate secondary joint movement
	// Eigen::MatrixXd J1_Bar = J1.completeOrthogonalDecomposition().pseudoInverse();
	Eigen::JacobiSVD<Eigen::MatrixXd> J1_Bar(J1, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd pJ1_J1 = J1_Bar.solve(J1);
	// Eigen::MatrixXd pJ1_J1 = J1_Bar*J1;
	Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(pJ1_J1.rows(), pJ1_J1.cols());
	Eigen::MatrixXd N1 = Identity - pJ1_J1;

	// Eigen::MatrixXd pinv_J2_N1 = (J2*N1).completeOrthogonalDecomposition().pseudoInverse();	
	Eigen::JacobiSVD<Eigen::MatrixXd> pinv_J2_N1(J2*N1, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Eigen::MatrixXd J2_pinv_J1 = J2*J1_Bar;
	Eigen::MatrixXd J2_pinv_J1_x1dot = J2*J1_Bar.solve(dx1);
	
	Eigen::VectorXd dq2 = N1 * pinv_J2_N1.solve(dx2 - J2_pinv_J1_x1dot);
	// Eigen::VectorXd dq2 = N1 * calculate_dQ(J2, dx2);


	Eigen::VectorXd dq_tot = dq1 + dq2;
	Eigen::VectorXd Q_des = Q_cur + dq_tot;

	

	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);


	currentFocusLength[H] = (hPoint[1] - pHeadDesired).norm();
	currentFocusLength[RE] = (rePoint[1] - pRightEyeDesired).norm();
	currentFocusLength[LE] = (lePoint[1] - pLeftEyeDesired).norm();


	if (currentTrajectoryTime > tTime)
		movement_complete = true;

	// std::cout << J2 << std::endl;
	return Q_des;
}



//Pseudoinverse stuff
// Eigen::MatrixXd J1_Bar = J1.completeOrthogonalDecomposition().pseudoInverse();
// Eigen::MatrixXd pJ1_J1 = J1_Bar*J1;

//Pseudoinverse stuff
// Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
// std::cout << svd.solve(b) << std::endl;

