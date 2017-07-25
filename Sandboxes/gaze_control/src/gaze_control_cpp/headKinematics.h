#ifndef headKinematics_h
#define headKinematics_h
#include <map>
#include <string>
class headKinematics{
	public:
		double l0 = 0.3115;
		double l1 = 0.13849;
		double l2 = 0.12508;
		double l3 = 0.053;
		Eigen::RowVectorXd S0 = Eigen::VectorXd::Zero(6);
		Eigen::RowVectorXd S1 = Eigen::VectorXd::Zero(6);
		Eigen::RowVectorXd S2 = Eigen::VectorXd::Zero(6); 
		Eigen::RowVectorXd S3 = Eigen::VectorXd::Zero(6); 
		Eigen::RowVectorXd S4 = Eigen::VectorXd::Zero(6); 
		Eigen::RowVectorXd S5 = Eigen::VectorXd::Zero(6); 
		Eigen::RowVectorXd S6 = Eigen::VectorXd::Zero(6);  


		Eigen::Matrix3d rHeadHome = Eigen::MatrixXd::Identity(3,3);
		Eigen::Matrix3d rRightEyeHome = Eigen::MatrixXd::Identity(3,3);
		Eigen::Matrix3d rLeftEyeHome = Eigen::MatrixXd::Identity(3,3);



		// Initialized within the constructor
		Eigen::Vector3d pHeadHome = Eigen::VectorXd::Zero(3);
		Eigen::Vector3d pRightEyeHome= Eigen::VectorXd::Zero(3);
		Eigen::Vector3d pLeftEyeHome= Eigen::VectorXd::Zero(3);
		Eigen::MatrixXd screwAxisTables = Eigen::MatrixXd::Zero(7,6);

		int J_num = screwAxisTables.rows();
		Eigen::RowVectorXd Jlist = Eigen::VectorXd::Zero(screwAxisTables.rows());

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