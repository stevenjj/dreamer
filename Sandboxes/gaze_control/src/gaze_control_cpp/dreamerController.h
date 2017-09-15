#ifndef dreamerController_h
#define dreamerController_h
#include <Eigen/Dense>
#include <cstring>
#include <map>

#include "ros/ros.h"
#include "headKinematics.h"
#include "dreamerJointPublisher.h"
Eigen::Matrix3d Rot(const Eigen::Vector3d&, const double);
Eigen::MatrixXd calculate_dQ(const Eigen::MatrixXd&, const Eigen::MatrixXd&);


class dreamerController{
	public:
		std::string H;
		std::string RE;
		std::string LE;

		bool movement_complete;

		headKinematics kinematics;
		dreamerJointPublisher pub;
		// peopleManager
		
		Eigen::MatrixXd jointJacobianConstraint;
		Eigen::VectorXd jointLimitMax;
		Eigen::VectorXd jointLimitMin;
		Eigen::VectorXd beta;
		Eigen::VectorXd jointActivationPos;
		Eigen::VectorXd jointActivationNeg;

		Eigen::VectorXi bufferRegionType;
		Eigen::MatrixXd IntermediateHMatrix;


		std::map <std::string, Eigen::RowVector3d> focusPointInit;
		std::map <std::string, double> currentFocusLength;
		std::map <std::string, Eigen::RowVector3d> gazePosition;
		std::map <std::string, Eigen::RowVector3d> gazeOldPosition;
		std::map <std::string, Eigen::RowVector3d> gazeVelocity;

		ros::NodeHandle handlerLow;
		ros::Publisher gazePublisher;
		
		dreamerController(void);
		~dreamerController(void);

		void initIntermediateTaskMatrices(void);
		void updateIntermediateHMatrix(void);

		void resetGazeFocus(void);
		void updateGazeFocus(const double);
		void publishFocusLength(void);
		bool gazeAreEyesFocused(void);
		double minJerkTimeScaling(const double, const double);
		void initializeHeadEyeFocusPoint(const Eigen::Vector3d&, const Eigen::Vector3d&);
		
		Eigen::Matrix3d calcSmoothDesiredOrientation(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);
		Eigen::Vector3d smoothOrientationError(const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);
		Eigen::VectorXd constantVelocityLookAtPoint(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const double);

		Eigen::VectorXd headPriorityEyeTrajectoryLookAtPoint(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const double, const double);
		Eigen::VectorXd eyePriorityHeadTrajectoryLookAtPoint(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const double, const double);
};

#endif