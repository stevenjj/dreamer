/**
 * Waypoint.h
 * Provides a wrapper to contain a point, a time, and a special parameter
 */

#ifndef Waypoint_h
#define Waypoint_h
#include <Eigen/Dense>
class Waypoint{
public:
	Eigen::Vector3d point; // Vector of point
	double Dt; // Change in time
	double special; // Special parameter (blink or head tilt)

	// Default constructor
	Waypoint(void);
	// Eigen Vector Constructor
	Waypoint(const Eigen::Vector3d, const double);
	//Eigen Vector Constructor with a special parameter
	Waypoint(const Eigen::Vector3d, const double, const double);
	// Individually define each parameter of the eigen vector
	Waypoint(const double, const double, const double, const double);
	// Individually define with a special parameter
	Waypoint(const double, const double, const double, const double, const double);
	// Specify only the position, velocity and accel = 0
	Waypoint(const double, const double);
	// Specify only the position, velocity and accel = 0 with a special parameter
	Waypoint(const double, const double, const double);


	~Waypoint(void);
};

#endif
