#ifndef headKinematics_h
#define headKinematics_h
#include <Eigen/Dense>
#include <map>
#include <string>
class headKinematics{
	public:
		double l0;
		double l1;
		double l2;
		double l3;
		
		Eigen::RowVectorXd S0;
		Eigen::RowVectorXd S1;
		Eigen::RowVectorXd S2; 
		Eigen::RowVectorXd S3; 
		Eigen::RowVectorXd S4; 
		Eigen::RowVectorXd S5; 
		Eigen::RowVectorXd S6;  


		Eigen::Matrix3d rHeadHome;
		Eigen::Vector3d pHeadHome;

		Eigen::Matrix3d rRightEyeHome;
		Eigen::Vector3d pRightEyeHome;
		
		Eigen::Matrix3d rLeftEyeHome;
		Eigen::Vector3d pLeftEyeHome;

		Eigen::MatrixXd screwAxisTables;

		int J_num;
		Eigen::RowVectorXd Jlist;

		std::map <int, std::string> JIndexToNames;

		headKinematics(void);

		~headKinematics(void);

		Eigen::MatrixXd get6D_HeadJacobian(const Eigen::VectorXd&);

		Eigen::MatrixXd get6D_RightEyeJacobian(const Eigen::VectorXd&);

		Eigen::MatrixXd get6D_LeftEyeJacobian(const Eigen::VectorXd&);


		Eigen::MatrixXd get6D_RightEyeJacobianYawPitch(const Eigen::VectorXd&);

		Eigen::MatrixXd get6D_LeftEyeJacobianYawPitch(const Eigen::VectorXd&);


		Eigen::MatrixXd* get6D_HeadPosition(const Eigen::VectorXd&);

		Eigen::MatrixXd* get6D_RightEyePosition(const Eigen::VectorXd&);

		Eigen::MatrixXd* get6D_LeftEyePosition(const Eigen::VectorXd&);

};
#endif