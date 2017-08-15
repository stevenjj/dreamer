#ifndef Waypoint_h
#define Waypoint_h
#include <Eigen/Dense>
class Waypoint{
public:
	Eigen::Vector3d point;
	double Dt;

	Waypoint(void);
	Waypoint(const Eigen::Vector3d, const double);
	~Waypoint(void);
};

#endif
