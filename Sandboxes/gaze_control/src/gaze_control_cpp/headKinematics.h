/*
 * headKinematics.h
 * Provides Functions for head-eye positions and jacobians
 * Keeps track of current 
 */

#ifndef headKinematics_h
#define headKinematics_h

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <cstring>

class headKinematics{
	public:
		// Declaration of Dreamer head joint lengths
		double l0;
		double l1;
		double l2;
		double l3;
		
		// Declaration of screw axis of each head-eye degree of freedom
		Eigen::RowVectorXd S0;
		Eigen::RowVectorXd S1;
		Eigen::RowVectorXd S2; 
		Eigen::RowVectorXd S3; 
		Eigen::RowVectorXd S4; 
		Eigen::RowVectorXd S5; 
		Eigen::RowVectorXd S6;  

		// Rotational matrix and positional vector of home position
		Eigen::Matrix3d rHeadHome;
		Eigen::Vector3d pHeadHome;

		Eigen::Matrix3d rRightEyeHome;
		Eigen::Vector3d pRightEyeHome;
		
		Eigen::Matrix3d rLeftEyeHome;
		Eigen::Vector3d pLeftEyeHome;


		// Combination of all screw axis declared above
		Eigen::MatrixXd screwAxisTables;

		// Number of joints and list of current joint positions
		int J_num;
		Eigen::RowVectorXd Jlist;

		// Map Joint to String
		std::map <int, std::string> JIndexToNames;

		headKinematics(void);
		~headKinematics(void);


		/* 
		 * Function: Gives the 6D Jacobian for Dreamer's head
		 * Inputs: Joint configuration
		 * Returns: 6D Spatial Head Jacobian
		 */
		Eigen::MatrixXd get6D_HeadJacobian(const Eigen::VectorXd&);
		

		/* 
		 * Function: Gives the 6D Jacobian for Dreamer's Right Eye
		 * Inputs: Joint configuration
		 * Returns: 6D Spatial Right Eye Jacobian
		 */
		Eigen::MatrixXd get6D_RightEyeJacobian(const Eigen::VectorXd&);
		

		/* 
		 * Function: Gives the 6D Jacobian for Dreamer's Left Eye
		 * Inputs: Joint configuration
		 * Returns: 6D Spatial Left Eye Jacobian
		 */
		Eigen::MatrixXd get6D_LeftEyeJacobian(const Eigen::VectorXd&);


		/* 
		 * Function: Gives the 6D Jacobian for Dreamer's Right Eye exlcuding head joints
		 * Inputs: Joint configuration
		 * Returns: 6D Spatial Jacobian
		 */
		Eigen::MatrixXd get6D_RightEyeJacobianYawPitch(const Eigen::VectorXd&);
		

		/* 
		 * Function: Gives the 6D Jacobian for Dreamer's Left Eye exlcuding head joints
		 * Inputs: Joint configuration
		 * Returns: 6D Spatial Jacobian
		 */
		Eigen::MatrixXd get6D_LeftEyeJacobianYawPitch(const Eigen::VectorXd&);




		/* 
		 * Function: Gives the spatial position of Dreamer's Head
		 * Inputs: Joint configuration
		 * Returns: Spatial position of the head
		 */
		std::vector<Eigen::MatrixXd> get6D_HeadPosition(const Eigen::VectorXd&);
		

		/* 
		 * Function: Gives the spatial position of Dreamer's Right Eye
		 * Inputs: Joint configuration
		 * Returns: Spatial position of the right eye
		 */
		std::vector<Eigen::MatrixXd> get6D_RightEyePosition(const Eigen::VectorXd&);
		
		
		/* 
		 * Function: Gives the spatial position of Dreamer's Left Eye
		 * Inputs: Joint configuration
		 * Returns: Spatial position of the left eye
		 */
		std::vector<Eigen::MatrixXd> get6D_LeftEyePosition(const Eigen::VectorXd&);

};
#endif