/*
 * dreamerController.cpp
 * Low level controller for HCRL Dreamer's head
 * Wrapper for head focus lengths/publisher
 * 
 * Does all low level calculations for desired joint configurations
 * 	given operational space points 
 */

#include <Eigen/Dense>
#include <iostream>
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
#include "dreamerJointPublisher.h"
#include "minJerk/minJerkCoordinates.h"

const double PI = M_PI;
const double FOCUS_THRESH = 0.22; // TODO find out why this isn't working
const double INIT_FOCUS_LENGTH = 0.6;
const double bufferRegionPercent = 0.05;

const int POS = 1;
const int NEG = -1;
const int NA = 0;

/*
 * Function: Deletes a specific row from a Matrix
 * Inputs: Matrix to remove row from, specific row to remove
 * Returns: Input matrix with specific row removed
 */
Eigen::MatrixXd deleteRow(const Eigen::MatrixXd& m, const int row){
	Eigen::MatrixXd m_ret(m.rows()-1, m.cols());
	std::vector<Eigen::RowVectorXd> vSlices;
	for (int k=0; k<7; k++){
		if(k != row)
			vSlices.push_back(m.row(k));
	}
	m_ret << vSlices[0],
			vSlices[1],
			vSlices[2],
			vSlices[3],
			vSlices[4],
			vSlices[5];
	return m_ret;
}

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
 * Inputs: Jacobian matrix for joint, change in operational space
 * Returns: Joint change
 * Note: Uses singular value decomposition for a pseudoinverse
 */
Eigen::MatrixXd calculate_dQ(const Eigen::MatrixXd& J, const Eigen::MatrixXd& dx){
	return J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dx);
}


Eigen::Vector3d cartToSphere(const Eigen::Vector3d& cartesian){
	double x = cartesian(0);
	double y = cartesian(1);
	double z = cartesian(2);
	double r = std::sqrt(x*x + y*y + z*z);
	double theta = std::atan(y/x);
	double phi = std::atan(std::sqrt(x*x+y*y)/z);
	Eigen::Vector3d s_ret(r, theta, phi);
	return s_ret;
}


// Generic Constructor
dreamerController::dreamerController(void){
	H = "head";
	RE = "right_eye";
	LE = "left_eye";

	movement_complete = false; // Variable becomes true when current task is completed


	// Focus states
	focusPointInit[H] << 1, 0, 0;
	focusPointInit[RE] << 1, 0, 0;
	focusPointInit[LE] << 1, 0, 0;

	currentFocusLength[H] = INIT_FOCUS_LENGTH;
	currentFocusLength[RE] = INIT_FOCUS_LENGTH;
	currentFocusLength[LE] = INIT_FOCUS_LENGTH;


	// Gaze motion variables
	// Initialize variables to zero in each std::map
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

	// Actually fill in correct gaze positions based on current position
	updateGazeFocus(1.0);

	// Fill in intermediate task stuff	
	jointLimitMax = Eigen::VectorXd::Zero(7);
	jointLimitMin = Eigen::VectorXd::Zero(7);
	beta = Eigen::VectorXd::Zero(7);
	jointActivationPos = Eigen::VectorXd::Zero(7);
	jointActivationNeg = Eigen::VectorXd::Zero(7);
	
	bufferRegionType = Eigen::VectorXi::Zero(7);
	IntermediateHMatrix = Eigen::MatrixXd::Zero(7,7);
	initIntermediateTaskMatrices();

	// Initialize gaze publisher
	// "gazePublisher" and "n" are defined in dreamerController.h
	gazePublisher = handlerLow.advertise<std_msgs::Float32MultiArray>("setArrLength", 1);

}


// Generic destructor
dreamerController::~dreamerController(void){}


void dreamerController::initIntermediateTaskMatrices(void){
	for(int i=0; i<kinematics.J_num; i++){
		std::string jointName = kinematics.JIndexToNames[i];
		jointLimitMax(i) = pub.freeJoints[jointName]["max"];
		jointLimitMin(i) = pub.freeJoints[jointName]["min"];
		
		double qRange = std::abs(jointLimitMax(i) - jointLimitMin(i));
		beta(i) = qRange * bufferRegionPercent;
		jointActivationPos(i) =  jointLimitMax(i) - beta(i);
		jointActivationNeg(i) =  jointLimitMin(i) - beta(i);
	}
}


void dreamerController::updateIntermediateHMatrix(void){
	for(int i=0; i<kinematics.J_num; i++){
		double q_i = kinematics.Jlist(i);
		double tilde_q_i = jointActivationPos(i);
		double utilde_q_i = jointActivationNeg(i);
		double bar_q_i = jointLimitMax(i);
		double ubar_q_i = jointLimitMin(i);
		double beta_i = beta(i);

		if(q_i >= bar_q_i){
			bufferRegionType(i) = POS;
			IntermediateHMatrix(i,i) = 1.0;
		}
		else if((tilde_q_i < q_i) && (q_i < bar_q_i)){
			bufferRegionType(i) = POS;
			IntermediateHMatrix(i,i) = 0.5 + 0.5*std::sin( (PI/beta_i) * (q_i - tilde_q_i) - PI/2.0);
		}
		else if((utilde_q_i <= q_i) && (q_i <= tilde_q_i)){
			bufferRegionType(i) = NA;
			IntermediateHMatrix(i,i) = 0;
		}
		else if((ubar_q_i < q_i) && (q_i < utilde_q_i)){
			bufferRegionType(i) = NEG;
			IntermediateHMatrix(i,i) = 0.5 + 0.5*std::sin((PI/beta_i)*(q_i - ubar_q_i) + PI/2.0);
		}
		else if(q_i <= ubar_q_i){
			bufferRegionType(i) = NEG;
			IntermediateHMatrix(i,i) = 1.0;
		}
		else{
			std::cout << "Error in IntermediateHMatrix" << std::endl;
			ros::shutdown();
		}
	}
}


/**
 * Function: Resets focus variables
 * Inputs: None
 * Returns: None
 */
void dreamerController::resetGazeFocus(void){
	focusPointInit[H] << 1, 0, 0;
	focusPointInit[RE] << 1, 0, 0;
	focusPointInit[LE] << 1, 0, 0;

	currentFocusLength[H] = INIT_FOCUS_LENGTH;
	currentFocusLength[RE] = INIT_FOCUS_LENGTH;
	currentFocusLength[LE] = INIT_FOCUS_LENGTH;

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
}



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
	double epsilon = .1 * PI/180.0;

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
 * Function to follow a minimum jerk curve
 * @param  headMin head minJerkCoordinates class
 * @param  eyesMin eyes minJerkCoordinates class
 * @param  sTime   start time of task in seconds
 * @return         desired joint configuration
 */
// TODO Eye priority motion doesn't work
Eigen::VectorXd dreamerController::headEyeTrajectoryFollow(minJerkCoordinates& headMin, minJerkCoordinates& eyesMin, const Eigen::VectorXd& initQ, const double sTime) {
	double totalRunTime = headMin.getTotalRunTime();
	double currentTime = ros::Time::now().toSec() - sTime;
	// Find the current desired points along the curve
	Eigen::Vector3d xyzHeadDesired = headMin.getPosition(currentTime);
	Eigen::Vector3d xyzEyesDesired = eyesMin.getPosition(currentTime);
	initializeHeadEyeFocusPoint(xyzHeadDesired, xyzEyesDesired);
	
	Eigen::VectorXd Q_cur = kinematics.Jlist;

	/************************** Head Calculations **************************/
	Eigen::MatrixXd J_head = kinematics.get6D_HeadJacobian(Q_cur);
	Eigen::MatrixXd J_head_block = J_head.block(0, 0, 3, J_head.cols());

	Eigen::Vector3d pHeadDesired = xyzHeadDesired;

	Eigen::Vector3d dxHead = smoothOrientationError(pHeadDesired, Q_cur, initQ, minJerkTimeScaling(currentTime, totalRunTime));
	
	/************************** Eyes Calculations **************************/
	Eigen::MatrixXd J1 = kinematics.get6D_RightEyeJacobian(Q_cur);
	Eigen::MatrixXd J2 = kinematics.get6D_LeftEyeJacobian(Q_cur);

	Eigen::MatrixXd J1_block = J1.block(1, 0, 2, J1.cols());
	Eigen::MatrixXd J2_block = J2.block(1, 0, 2, J2.cols());

	Eigen::MatrixXd J_eyes(J1_block.rows()+J2_block.rows(), J1_block.cols());
	J_eyes << J1_block,
			  J2_block;

	Eigen::Vector3d pRightEyeDesired = xyzEyesDesired;
	Eigen::Vector3d pLeftEyeDesired = xyzEyesDesired;


	Eigen::Vector3d dxRE = smoothOrientationError(pRightEyeDesired, Q_cur, initQ, minJerkTimeScaling(currentTime, totalRunTime), "right_eye");
	Eigen::Vector3d dxLE = smoothOrientationError(pLeftEyeDesired, Q_cur, initQ, minJerkTimeScaling(currentTime, totalRunTime), "left_eye");
	
	Eigen::Vector2d dxRE_block(dxRE(1), dxRE(2));
	Eigen::Vector2d dxLE_block(dxLE(1), dxLE(2));
	Eigen::MatrixXd dxEyes(dxRE_block.rows() + dxLE_block.rows(), dxRE_block.cols());

	dxEyes << dxRE_block,
			  dxLE_block;

	int HEAD = 1;
	int EYES = 2;
	int PRIORITY = EYES;


	Eigen::MatrixXd dx1;
	Eigen::MatrixXd dx2;

	if(PRIORITY == HEAD) {
		J1 = J_head_block;
		J2 = J_eyes;
		dx1 = dxHead;
		dx2 = dxEyes;
	}
	else {
		J1 = J_eyes;
		J2 = J_head_block;
		dx1 = dxEyes;
		dx2 = dxHead;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> J1_Bar(J1, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd pJ1_J1 = J1_Bar.solve(J1);
	
	Eigen::MatrixXd N1 = Eigen::MatrixXd::Identity(kinematics.J_num, kinematics.J_num) - pJ1_J1;
	Eigen::MatrixXd J2_N1 = J2*N1;


	Eigen::VectorXd dq1_proposed = J1_Bar.solve(dx1);
	
	Eigen::JacobiSVD<Eigen::MatrixXd> J2_N1_Bar(J2_N1, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd dq2_proposed = J2_N1_Bar.solve(dx2 - J2*dq1_proposed);








	Eigen::VectorXd dq_tot = dq1_proposed + dq2_proposed;
	Eigen::VectorXd Q_des = Q_cur + dq_tot;

	
	// Find current end effector positions and update the RVIZ gaze length
	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);


	currentFocusLength[H] = (hPoint[1] - pHeadDesired).norm();
	currentFocusLength[RE] = (rePoint[1] - pRightEyeDesired).norm();
	currentFocusLength[LE] = (lePoint[1] - pLeftEyeDesired).norm();

	// Note if the task is completed
	if (currentTime > totalRunTime)
		movement_complete = true;
	else
		movement_complete = false;

	return Q_des;


}



/**
 * Function: Calculate desired joint positions for go to point with head priority
 * Inputs: xyz head gaze, xyz eye gaze, joint configuration, motion start time, total motion run time
 * Returns: Desired joint configuation
 */
Eigen::VectorXd dreamerController::headPriorityEyeTrajectoryLookAtPoint(const Eigen::Vector3d& xyzHeadGaze, const Eigen::Vector3d& xyzEyeGaze, const Eigen::VectorXd& initQ, const double sTime, const double tTime){
	double currentTrajectoryTime = ros::Time::now().toSec();
	currentTrajectoryTime -= sTime;
	

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
		

	// Move towards the desired point along a minimum jerk
	Eigen::Vector3d pHeadDesired = headFocus.transpose() + error * lHead * minJerkTimeScaling(currentTrajectoryTime, tTime);

	// Find the operational space change
	Eigen::Vector3d dxHead = smoothOrientationError(pHeadDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime));
	

	/************************** Eyes Calculations **************************/
	// Similar to head calculations
	// We can append Jacobian matrices and operation space motions of the eyes
	// 		together because they are of the same priority
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
	


	// Calculate primary joint task
	Eigen::VectorXd dq1 = calculate_dQ(J1, dx1);
	

	// Calculate secondary joint task
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

	// Add desired joint changes to current configuration
	Eigen::VectorXd dq_tot = dq1 + dq2;
	Eigen::VectorXd Q_des = Q_cur + dq_tot;

	
	// Find current end effector positions and update the RVIZ gaze length
	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);


	currentFocusLength[H] = (hPoint[1] - pHeadDesired).norm();
	currentFocusLength[RE] = (rePoint[1] - pRightEyeDesired).norm();
	currentFocusLength[LE] = (lePoint[1] - pLeftEyeDesired).norm();

	// Note if the task is completed
	if (currentTrajectoryTime > tTime)
		movement_complete = true;
	else
		movement_complete = false;

	return Q_des;
}



/**
 * Function: Calculate desired joint positions for go to point with head priority at a specifc velocity
 * Inputs: xyz head gaze, xyz eye gaze, joint configuration, motion start time
 * Returns: Desired joint configuation
 * Done in Spherical coordinates
 * Assumes a relatively far distance from the head
 */
Eigen::VectorXd dreamerController::constantVelocityLookAtPoint(const Eigen::Vector3d& xyzHeadGaze, const Eigen::Vector3d& xyzEyeGaze, const Eigen::VectorXd& initQ, const double sTime){
	const double velocity = .25; // distance in cm/s
	double currentTrajectoryTime = ros::Time::now().toSec();
	currentTrajectoryTime -= sTime;
	

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

	double moveScale = currentTrajectoryTime / (lHead / velocity);

	// Move towards the desired point along a constant speed
	Eigen::Vector3d pHeadDesired = headFocus.transpose() + error * lHead * moveScale;

	// Find the operational space change
	Eigen::Vector3d dxHead = smoothOrientationError(pHeadDesired, Q_cur, initQ, moveScale);
	

	/************************** Eyes Calculations **************************/
	// Similar to head calculations
	// We can append Jacobian matrices and operation space motions of the eyes
	// 		together because they are of the same priority
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

	Eigen::Vector3d pRightEyeDesired = reFocus.transpose() + errorRE * lRE * moveScale;
	Eigen::Vector3d pLeftEyeDesired = leFocus.transpose() + errorLE * lLE * moveScale;


	Eigen::Vector3d dxRE = smoothOrientationError(pRightEyeDesired, Q_cur, initQ, moveScale, "right_eye");
	Eigen::Vector3d dxLE = smoothOrientationError(pLeftEyeDesired, Q_cur, initQ, moveScale, "left_eye");
	Eigen::MatrixXd dxEyes(dxRE.rows() + dxLE.rows(), dxRE.cols());

	dxEyes << dxRE,
			  dxLE;


	// Specify priority
	J1 = J_head_block;
	Eigen::MatrixXd dx1 = dxHead;
	J2 = J_eyes;
	Eigen::MatrixXd dx2 = dxEyes;
	


	// Calculate primary joint task
	Eigen::VectorXd dq1 = calculate_dQ(J1, dx1);
	

	// Calculate secondary joint task
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

	// Add desired joint changes to current configuration
	Eigen::VectorXd dq_tot = dq1 + dq2;
	Eigen::VectorXd Q_des = Q_cur + dq_tot;

	
	// Find current end effector positions and update the RVIZ gaze length
	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);


	currentFocusLength[H] = (hPoint[1] - pHeadDesired).norm();
	currentFocusLength[RE] = (rePoint[1] - pRightEyeDesired).norm();
	currentFocusLength[LE] = (lePoint[1] - pLeftEyeDesired).norm();


	std::vector<Eigen::MatrixXd> head = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> re = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> le = kinematics.get6D_LeftEyePosition(kinematics.Jlist);

	Eigen::Vector3d xhead = head[0].block<3,1>(0, 0);
	Eigen::Vector3d xre = re[0].block<3,1>(0, 0);
	Eigen::Vector3d xle = le[0].block<3,1>(0, 0);

	return Q_des;
}


/**
 * Provides desired joint configurations assuming eye priority control
 * @param  xyzHeadGaze desired head gaze location
 * @param  xyzEyeGaze  desired eyes gaze location
 * @param  initQ       initial head joint configuration
 * @param  sTime       start time of the task in seconds
 * @param  tTime       total run time of task in seconds
 * @return             desired joint configuration
 */
Eigen::VectorXd dreamerController::eyePriorityHeadTrajectoryLookAtPoint(const Eigen::Vector3d& xyzHeadGaze, const Eigen::Vector3d& xyzEyeGaze, const Eigen::VectorXd& initQ, const double sTime, const double tTime){
	double currentTrajectoryTime = ros::Time::now().toSec();
	currentTrajectoryTime -= sTime;
	

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
		

	// Move towards the desired point along a minimum jerk
	Eigen::Vector3d pHeadDesired = headFocus.transpose() + error * lHead * minJerkTimeScaling(currentTrajectoryTime, tTime);

	// Find the operational space change
	Eigen::Vector3d dxHead = smoothOrientationError(pHeadDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime));
	

	/************************** Eyes Calculations **************************/
	// Similar to head calculations
	// We can append Jacobian matrices and operation space motions of the eyes
	// 		together because they are of the same priority
	Eigen::MatrixXd J1 = kinematics.get6D_RightEyeJacobian(Q_cur);
	Eigen::MatrixXd J2 = kinematics.get6D_LeftEyeJacobian(Q_cur);

	// Grab rows 2 and 3 of each Jacobian Matrix
	Eigen::MatrixXd J1_block = J1.block(1, 0, 2, J1.cols());
	Eigen::MatrixXd J2_block = J2.block(1, 0, 2, J2.cols());

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
	Eigen::Vector2d dxRE_block(dxRE(1), dxRE(2));
	Eigen::Vector3d dxLE = smoothOrientationError(pLeftEyeDesired, Q_cur, initQ, minJerkTimeScaling(currentTrajectoryTime, tTime), "left_eye");
	Eigen::Vector2d dxLE_block(dxLE(1), dxLE(2));
	Eigen::MatrixXd dxEyes(dxRE_block.rows() + dxLE_block.rows(), dxRE_block.cols());

	dxEyes << dxRE_block,
			  dxLE_block;


	// Specify priority
	J1 = J_eyes;
	Eigen::MatrixXd dx1 = dxEyes;
	J2 = J_head_block;
	Eigen::MatrixXd dx2 = dxHead;
	


	/***************** Calculate joint task *****************/
	Eigen::VectorXd dq1_proposed = calculate_dQ(J1, dx1);
	

	// Calculate secondary joint task
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
	
	Eigen::VectorXd dq2_proposed = N1 * pinv_J2_N1.solve(dx2 - J2_pinv_J1_x1dot);
	// Eigen::VectorXd dq2 = N1 * calculate_dQ(J2, dx2);



	/*
	// Intermediate task Calculations
	updateIntermediateHMatrix();
	// std::cout << "IntermediateHMatrix: \n" << IntermediateHMatrix << std::endl;
	Eigen::VectorXd x0_d = Eigen::VectorXd::Zero(kinematics.J_num);
	for(int i=0; i<kinematics.J_num; i++){
		double q_i = Q_cur(i);
		switch(bufferRegionType(i)){
			case POS: x0_d(i) = .01 * (0-q_i); break;
			case NEG: x0_d(i) = .01 * (0-q_i); break;
			case NA: x0_d(i) = 0; break;
			default: std::cout << "error in buffer region" << std::endl; ros::shutdown(); break;
		}
	}


	Eigen::VectorXd dx0_i = Eigen::VectorXd::Zero(kinematics.J_num);
	Eigen::MatrixXd J0_constraint = Eigen::MatrixXd::Identity(7,7);
	for(int j=0; j< kinematics.J_num; j++){
		Eigen::RowVectorXd J0_j = J0_constraint.block(j, 0, 1, J0_constraint.cols());
		
		// Remove the specific joint
		Eigen::MatrixXd J0_wj = deleteRow(J0_constraint, j);
		
		// Null Space of constraints
		Eigen::JacobiSVD<Eigen::MatrixXd> pinvJ0_wj(J0_wj, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd Ident = Eigen::MatrixXd::Identity(J0_constraint.rows(), J0_constraint.cols());
		Eigen::MatrixXd N0_wj = Ident - pinvJ0_wj.solve(J0_wj);
	
		Eigen::JacobiSVD<Eigen::MatrixXd> pinvJ1_N0_wj(J1 * N0_wj, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd N1_0_wj = Ident - pinvJ1_N0_wj.solve(J1 * N0_wj);

		Eigen::VectorXd x0_d_wj = deleteRow(x0_d, j);

		Eigen::VectorXd dq0_wj = pinvJ0_wj.solve(x0_d_wj);
		// Method 1
		// Eigen::VectorXd dq1_wj = J1_Bar.solve(dx1);
		// Eigen::VectorXd dq2_wj = pinv_J2_N1.solve(dx2*.05 - J2*dq1_wj);
		Eigen::VectorXd dq1_wj = (J1 * N0_wj).transpose() * (dx1 - (J1 * dq0_wj));
		Eigen::JacobiSVD<Eigen::MatrixXd> pinvJ2_N0_wj_N1_0_wj(J2 * N0_wj * N1_0_wj, Eigen::ComputeThinU | Eigen::ComputeThinV);
		
		Eigen::VectorXd dq2_wj = pinvJ2_N0_wj_N1_0_wj.solve(dx2 - (J2 * (dq1_wj + dq0_wj)));
		
		Eigen::VectorXd dq_wj = dq0_wj + dq1_wj + dq2_wj;
		// std::cout << "dq0wj: \n" << dq0_wj << std::endl;
		// std::cout << "dq1wj: \n" << dq1_wj << std::endl;
		// std::cout << "dq2wj: \n" << dq2_wj << std::endl;
		double h_j = IntermediateHMatrix(j, j);

		if(j<4){
			double hEyeMax = h_j;
			for(int k=4; k<kinematics.J_num; j++){
				double hCandidate = IntermediateHMatrix(k, k);
				if( hCandidate >= hEyeMax)
					hEyeMax = hCandidate;
			}
			h_j = hEyeMax;
		}

		dx0_i(j) = h_j*(x0_d(j)) + (1 - h_j) * (J0_j) * dq_wj;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> pinvJConstraint(J0_constraint, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd dq0 = pinvJConstraint.solve(dx0_i);
*/
	// Add desired joint changes to current configuration
	Eigen::VectorXd dq_tot = dq1_proposed + dq2_proposed;
	Eigen::VectorXd Q_des = Q_cur + dq_tot;

	
	// Find current end effector positions and update the RVIZ gaze length
	std::vector<Eigen::MatrixXd> hPoint = kinematics.get6D_HeadPosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> rePoint = kinematics.get6D_RightEyePosition(kinematics.Jlist);
	std::vector<Eigen::MatrixXd> lePoint = kinematics.get6D_LeftEyePosition(kinematics.Jlist);


	currentFocusLength[H] = (hPoint[1] - pHeadDesired).norm();
	currentFocusLength[RE] = (rePoint[1] - pRightEyeDesired).norm();
	currentFocusLength[LE] = (lePoint[1] - pLeftEyeDesired).norm();

	// Note if the task is completed
	if (currentTrajectoryTime > tTime)
		movement_complete = true;
	else
		movement_complete = false;

	return Q_des;
}



//Pseudoinverse stuff
// Eigen::MatrixXd J1_Bar = J1.completeOrthogonalDecomposition().pseudoInverse();
// Eigen::MatrixXd pJ1_J1 = J1_Bar*J1;

//Pseudoinverse stuff
// Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
// std::cout << svd.solve(b) << std::endl;

