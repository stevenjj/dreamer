#ifndef _modernRobotics_h
#define _modernRobotics_h

bool NearZero(double);
Eigen::MatrixXd Normalize(Eigen::MatrixXd);
Eigen::Matrix3d VecToso3(Eigen::Vector3d);
Eigen::Vector3d so3ToVec(Eigen::MatrixXd);
Eigen::Vector4d AxisAng3(Eigen::Vector3d);
Eigen::Matrix3d MatrixExp3(Eigen::Matrix3d);
Eigen::MatrixXd RpToTrans(Eigen::Matrix3d, Eigen::Vector3d);
Eigen::MatrixXd* TransToRp(Eigen::MatrixXd);
Eigen::MatrixXd VecTose3(Eigen::VectorXd);
Eigen::MatrixXd Adjoint(Eigen::MatrixXd);
Eigen::MatrixXd MatrixExp6(Eigen::MatrixXd);
Eigen::MatrixXd FKinSpace(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd);
Eigen::MatrixXd JacobianSpace(Eigen::MatrixXd ,Eigen::MatrixXd);


#endif