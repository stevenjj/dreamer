#ifndef _modernRobotics_h
#define _modernRobotics_h
#include <Eigen/Dense>
bool NearZero(const double);
Eigen::MatrixXd Normalize(Eigen::MatrixXd);
Eigen::Matrix3d VecToso3(const Eigen::Vector3d&);
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd&);
Eigen::Vector4d AxisAng3(const Eigen::Vector3d&);
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d&);
Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d&, const Eigen::Vector3d&);
Eigen::MatrixXd* TransToRp(const Eigen::MatrixXd&);
Eigen::MatrixXd VecTose3(const Eigen::VectorXd&);
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd&);
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd&);
Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::VectorXd&);
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
#endif