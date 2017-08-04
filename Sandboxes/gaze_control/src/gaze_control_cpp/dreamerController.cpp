#include <Eigen/Dense>
#include <iostream>
#include <ctime>
#include <cstring>
#include <stdexcept>
#include <cmath>

#include "ros/ros.h"
#include "modernRobotics.h"
#include "headKinematics.h"
#include "utilQuat.h"
#include "dreamerController.h"

headKinematics hk;
const double pi = M_PI;

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


/* Function: Calculate a rotation matrix based on the desired parameters
 * Inputs: desired gaze location, current spatial position, desired joint configuration, initial join configuration, min_jerk scaling, head component, head roll
 * Returns: Desired Rotation Matrix
 */
Eigen::Matrix3d calcSmoothDesiredOrientation(const Eigen::Vector3d& xGazeLoc, const Eigen::Vector3d& pCur, const Eigen::VectorXd& Q_cur, const Eigen::VectorXd& Q_init, const double scaling, const std::string orientationType = "head", const double tilt = 0) {
	// Find unit vector of the difference between desired position and current position
	Eigen::Vector3d pBar = xGazeLoc - pCur;
	Eigen::Vector3d xHatD = Normalize(pBar);
	// std::cout << pCur << std::endl;

	// Get Rotation and position of the component
	Eigen::MatrixXd *pos;
	if(orientationType == "head")
		pos = hk.get6D_HeadPosition(Q_init);
	else if(orientationType == "right_eye")
		pos = hk.get6D_RightEyePosition(Q_init);
	else if(orientationType == "left_eye")
		pos = hk.get6D_LeftEyePosition(Q_init);
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
	delete [] pos;
	return RDesired; 
}


/* Function: Calculates error between current and desired configurations
 * Inputs: desired gaze location, desired joint configuration, current joint configuration, min_jerk time scaling
 * Returns: Error in angle * angular velocities as an array
 */
Eigen::Vector3d smoothOrientationError(const Eigen::Vector3d& xGazeLoc, const Eigen::VectorXd& Q, const Eigen::VectorXd& Q_init, const double scaling, const std::string orientationType = "head", const double tilt = 0){
	Eigen::MatrixXd *pos;
	if(orientationType == "head")
		pos = hk.get6D_HeadPosition(Q);
	else if(orientationType == "right_eye")
		pos = hk.get6D_RightEyePosition(Q);
	else if(orientationType == "left_eye")
		pos = hk.get6D_LeftEyePosition(Q);
	else
		throw std::invalid_argument("Unknown position and orientation needed");

	Eigen::Matrix3d R_des = calcSmoothDesiredOrientation(xGazeLoc, pos[1], Q, Q_init, scaling, orientationType, tilt);

	Eigen::RowVector4d qCur = RToQuat(pos[0]);
	Eigen::RowVector4d qDes = RToQuat(R_des);

	Eigen::RowVector4d qError = quatMultiply(qDes, conj(qCur));
	delete [] pos;
	return quatToWth(qError);
}



dreamerController::dreamerController(void){}

dreamerController::~dreamerController(void){}




/**
 * Function: Calculate desired joint positions for go to point with head priority
 * Inputs: xyz head gaze, xyz eye gaze, start time, total run time
 * 
 */
Eigen::MatrixXd dreamerController::headPriorityEyeTrajectoryLookAtPoint(Eigen::Vector3d xyzHeadGaze, Eigen::Vector3d xyzEyeGaze, const double stime, const double ttime){
	Eigen::MatrixXd m_ret; // dummy return
	double currentTrajectoryTime = ros::Time::now().toSec() - stime;
	std::cout << currentTime << std::endl;
	return m_ret;
}



int main(void){
	ros::Time::init();
	std::cout << "Beginning ROS" << std::endl;
	double init_time = ros::Time::now().toSec();




	// Eigen::MatrixXd *point1 = hk.get6D_LeftEyePosition(hk.Jlist);
	// Eigen::MatrixXd *point2 = hk.get6D_RightEyePosition(hk.Jlist);
	// Eigen::MatrixXd *point3 = hk.get6D_HeadPosition(hk.Jlist);
	// std::cout << point1[1] << std::endl;
	// std::cout << point2[1] << std::endl;
	// std::cout << point3[1] << std::endl;
	// delete [] point1;
	// delete [] point2;
	// delete [] point3;



	// for(int i=0; i<1000; i++) {
	// }
	// std::cout << "Time Taken: " << (ros::Time::now().toSec() - init_time) << " ms" << std::endl;

	// double time = 0;
	//    for(int i = 0; i < 1000; i++) {
	// 	std::clock_t    start;
	//    	start = std::clock();
	// 	Eigen::MatrixXd *point1 = hk.get6D_LeftEyePosition(hk.Jlist);
	// 	time+=(std::clock() - start) / (double)(CLOCKS_PER_SEC/1000);
	// }
	//    std::cout << "Time: " << (time/1000) << " ms" << std::endl;
	// return -1;

}


//Pseudoinverse stuff
// Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
// std::cout << svd.solve(b) << std::endl;

