#ifndef utilQuat_h
#define utilQuat_h
//#include <Eigen/Dense>
Eigen::RowVector4d wthToQuat(const Eigen::RowVector3d&, double);
Eigen::RowVector3d quatToWth(Eigen::RowVector4d);
Eigen::RowVector4d RToQuat(Eigen::Matrix3d);
Eigen::Matrix3d quatToR(Eigen::RowVector4d);
Eigen::RowVector4d quatMultiply(Eigen::RowVector4d, Eigen::RowVector4d);
Eigen::RowVector4d conj(Eigen::RowVector4d);

#endif