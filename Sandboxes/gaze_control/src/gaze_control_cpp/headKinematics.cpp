#include <Eigen/Dense>
#include <iostream>
#include "modernRobotics.h"
#include <map>
#include <ctime>
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

		headKinematics(void){
			S0 << 0, -1, 0, 0, 0, 0; 
			S1 << 0, 0, 1, 0, 0, 0;
			S2 << -1, 0, 0, 0, -l1, 0;  
			S3 << 0, -1, 0, l1, 0, 0;  
			S4 << 0, -1, 0, l1, 0, -l2;  
			S5 << 0, 0, 1, -l3, -l2, 0;  
			S6 << 0, 0, 1, l3, -l2, 0;  


			pHeadHome << 0, 0, l1;
			pRightEyeHome << l2, -l3, l1;
			pLeftEyeHome << l2, l3, l1;
			
			screwAxisTables << S0, S1, S2, S3, S4, S5, S6;

			JIndexToNames[0] = "lowerNeckPitch";
			JIndexToNames[1] = "upperNeckYaw";
			JIndexToNames[2] = "upperNeckRoll";
			JIndexToNames[3] = "upperNeckPitch";
			JIndexToNames[4] = "eyePitch";
			JIndexToNames[5] = "rightEyeYaw";
			JIndexToNames[6] = "leftEyeYaw";
			
		}

		~headKinematics(void){};



		Eigen::MatrixXd get6D_HeadJacobian(Eigen::MatrixXd JList){
			const int screwAxisEnd = 3;
			const int numJoints = screwAxisEnd + 1;
			Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
			Eigen::RowVectorXd thetaList = Jlist.head(numJoints); // First 4 joints
			Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


			Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
			Eigen::MatrixXd JSpatial_6DHead(6,7);

			JSpatial_6DHead << JSpatial, zeroVec;

			return JSpatial_6DHead;
		}

		Eigen::MatrixXd get6D_RightEyeJacobian(Eigen::MatrixXd JList){
			const int screwAxisEnd = 5;
			const int numJoints = screwAxisEnd + 1;
			Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
			Eigen::RowVectorXd thetaList = Jlist.head(numJoints); // First 6 joints
			Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


			Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
			Eigen::MatrixXd JSpatial_6DRightEye(6,7);

			JSpatial_6DRightEye << JSpatial, zeroVec;

			return JSpatial_6DRightEye;
		}


		Eigen::MatrixXd get6D_LeftEyeJacobian(Eigen::MatrixXd JList){
			const int screwAxisEnd = 6;
			const int numJoints = screwAxisEnd;
			Eigen::MatrixXd Slist(6,6);
			Slist << screwAxisTables.block<5, 6>(0,0), screwAxisTables.block<1, 6>(6,0);

			Eigen::RowVectorXd thetaList(6);
			thetaList << Jlist.head(numJoints-1), Jlist(numJoints);
			
			Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);

			Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, J_num - numJoints);
			Eigen::MatrixXd JSpatial_6DLeftEye(6,7);

			// First 5 columns, 0 column, last column
			JSpatial_6DLeftEye << JSpatial.block<6,5>(0,0), zeroVec, JSpatial.block<6,1>(0,5);

			return JSpatial_6DLeftEye;
		}


		Eigen::MatrixXd get6D_RightEyeJacobianYawPitch(Eigen::MatrixXd JList){
			const int screwAxisEnd = 5;
			const int numJoints = screwAxisEnd + 1;
			Eigen::MatrixXd Slist = screwAxisTables.block<2, 6>(4,0);
			Eigen::RowVectorXd thetaList = Jlist.segment(4,2); // 2 joint starting from joint 4
			Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);


			Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, 4);
			Eigen::MatrixXd JSpatial_6DRightEye(6,7);

			JSpatial_6DRightEye << zeroVec, JSpatial, zeroVec.block<6,1>(0,0);

			return JSpatial_6DRightEye;
		}

		Eigen::MatrixXd get6D_LeftEyeJacobianYawPitch(Eigen::MatrixXd JList){
			const int screwAxisEnd = 6;
			const int numJoints = screwAxisEnd;
			Eigen::MatrixXd Slist(2,6);
			Slist << screwAxisTables.block<1, 6>(4,0), screwAxisTables.block<1, 6>(6,0);
			Eigen::RowVector2d thetaList(Jlist(4), Jlist(6)); 
			Eigen::MatrixXd JSpatial = JacobianSpace(Slist.transpose(), thetaList);
			std::cout << JSpatial << std::endl;

			Eigen::MatrixXd zeroVec = Eigen::MatrixXd::Zero(6, 4);
			Eigen::MatrixXd JSpatial_6DLeftEye(6,7);

			// First 4 columns of zeros, 1 column, zero column, last column
			JSpatial_6DLeftEye << zeroVec, JSpatial.block<6,1>(0,0) , zeroVec.block<6,1>(0,0),  JSpatial.block<6,1>(0,1);

			return JSpatial_6DLeftEye;
		}


		Eigen::MatrixXd* get6D_HeadPosition(Eigen::MatrixXd JList){
			const int screwAxisEnd = 3;
			const int numJoints = screwAxisEnd + 1;
			Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
			Eigen::RowVectorXd thetaList = Jlist.head(numJoints); // First 4 joints

			Eigen::MatrixXd M_HeadHome = RpToTrans(rHeadHome, pHeadHome);
			
			Eigen::MatrixXd T_Head = FKinSpace(M_HeadHome, Slist.transpose(), thetaList);

			return TransToRp(T_Head);
		}


		Eigen::MatrixXd* get6D_RightEyePosition(Eigen::MatrixXd JList){
			const int screwAxisEnd = 5;
			const int numJoints = screwAxisEnd + 1;
			Eigen::MatrixXd Slist = screwAxisTables.block<numJoints, 6>(0,0);
			Eigen::RowVectorXd thetaList = Jlist.head(numJoints); // First 4 joints

			Eigen::MatrixXd M_RightEyeHome = RpToTrans(rRightEyeHome, pRightEyeHome);
			
			Eigen::MatrixXd T_RightEye = FKinSpace(M_RightEyeHome, Slist.transpose(), thetaList);

			return TransToRp(T_RightEye);
		}


		Eigen::MatrixXd* get6D_LeftEyePosition(Eigen::MatrixXd JList){
			const int screwAxisEnd = 6;
			const int numJoints = screwAxisEnd;

			Eigen::MatrixXd Slist(6,6);
			Slist << screwAxisTables.block<5, 6>(0,0), screwAxisTables.block<1, 6>(6,0);

			Eigen::RowVectorXd thetaList(6);
			thetaList << Jlist.head(numJoints-1), Jlist(numJoints);

			Eigen::MatrixXd M_LeftEyeHome = RpToTrans(rLeftEyeHome, pLeftEyeHome);
			
			Eigen::MatrixXd T_LeftEye = FKinSpace(M_LeftEyeHome, Slist.transpose(), thetaList);

			return TransToRp(T_LeftEye);
		}





};

int main(){
	headKinematics hk;
	hk.Jlist << 2,4,6,8,10,12,14;
	std::cout << hk.get6D_LeftEyePosition(hk.Jlist)[0] << std::endl;
	std::cout << hk.get6D_LeftEyePosition(hk.Jlist)[1] << std::endl;

	// std::clock_t start;
	// start = std::clock();
	// for(int i=0; i<100; i++){
 //    }
 //    double time = (std::clock() - start) / (double)(CLOCKS_PER_SEC) / 100.0;
	// std::cout << "Time per loop: " << time << " s" << std::endl;


	return -1;
}	