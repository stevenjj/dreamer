#ifndef dreamerController_h
#define dreamerController_h
#include <Eigen/Dense>
#include <cstring>
#include <map>
#include "headKinematics.h"
Eigen::Matrix3d Rot(const Eigen::Vector3d&, const double);
Eigen::MatrixXd calculate_dQ(const Eigen::MatrixXd&, const Eigen::MatrixXd&);


class dreamerController{
	public:
		std::string H;
		std::string RE;
		std::string LE;

		headKinematics kinematics;
		// gazeFocusStates
		// peopleManager
		// jointPublisher
		
		Eigen::MatrixXd jointJacobianConstraint;

		std::map <std::string, Eigen::RowVector3d> focusPointInit;
		std::map <std::string, double> currentFocusLength;
		std::map <std::string, Eigen::RowVector3d> gazePosition;
		std::map <std::string, Eigen::RowVector3d> gazeOldPosition;
		std::map <std::string, Eigen::RowVector3d> gazeVelocity;


		dreamerController(void);
		~dreamerController(void);

		void updateGazeFocus(const double);
		bool gazeAreEyesFocused(void);
		double minJerkTimeScaling(const double, const double);
		Eigen::Matrix3d calcSmoothDesiredOrientation(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);
		Eigen::Vector3d smoothOrientationError(const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);
		Eigen::VectorXd headPriorityEyeTrajectoryLookAtPoint(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const double, const double);
};

#endif