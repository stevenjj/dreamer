/**
 * Waypoint.h
 * Provides a wrapper to contain a point, a time, and a special parameter
 */

#ifndef Waypoint_h
#define Waypoint_h
#include <Eigen/Dense>
class Waypoint{
public:
	Eigen::Vector3d point;
	double Dt;
	double special;

	Waypoint(void);
	Waypoint(const Eigen::Vector3d, const double);
	Waypoint(const Eigen::Vector3d, const double, const double);
	~Waypoint(void);
};

#endif
