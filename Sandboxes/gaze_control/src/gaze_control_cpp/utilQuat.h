/* 
 * utilQuat.h
 * Extension of modernRobotics software
 * Provides quaternion functionality
 */

#ifndef utilQuat_h
#define utilQuat_h

#include <Eigen/Dense>

/* 
 * Function: Change an exponential representation to quaternion
 * Inputs: angular velocity vector, rotation about that vector
 * Returns: Quaternion
 */
Eigen::RowVector4d wthToQuat(const Eigen::RowVector3d&, double);


/* 
 * Function: Changes a quaternion to the exponential representation
 * Inputs: Quaternion
 * Returns: Exponential representation
 */
Eigen::RowVector3d quatToWth(const Eigen::RowVector4d&);


/* 
 * Function: Converts a rotation matrix to a quaternion
 * Inputs: Rotation matrix
 * Returns: Quaternion
 */
Eigen::RowVector4d RToQuat(const Eigen::Matrix3d&);


/* 
 * Function: Converts a quaternion to a rotation matrix
 * Inputs: Quaternion
 * Returns: Rotation matrix
 */
Eigen::Matrix3d quatToR(const Eigen::RowVector4d&);


/* 
 * Function: Provides the unit quaternion product of two quaternions
 * Inputs: 2 Quaternions
 * Returns: Unit quaternion product
 * Notes: Copy needed
 */
Eigen::RowVector4d quatMultiply(Eigen::RowVector4d, Eigen::RowVector4d);


/* 
 * Function: Returns the conjugate of a unit quaternion
 * Inputs: Quaternion
 * Returns: Inverse/conjugate of the quaternion
 * Notes: Copy needed
 */
Eigen::RowVector4d conj(Eigen::RowVector4d);

#endif
