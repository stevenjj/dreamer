#ifndef dreamerController_h
#define dreamerController_h
#include <Eigen/Dense>
#include <cstring>

#include "headKinematics.h"
Eigen::Matrix3d Rot(const Eigen::Vector3d&, const double);
Eigen::MatrixXd calculate_dQ(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
Eigen::Matrix3d calcSmoothDesiredOrientation(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);
Eigen::Vector3d smoothOrientationError(const Eigen::Vector3d&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double, const std::string, const double);


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
		dreamerController(void);
		~dreamerController(void);

		Eigen::MatrixXd headPriorityEyeTrajectoryLookAtPoint(const double);
};

#endif