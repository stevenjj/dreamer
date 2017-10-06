/**
 * Waypoint.cpp
 * Provides a wrapper to contain a point with 
 * 	(point, velocity, acceleration), a time, special parameter (tilt or blinking)
 */

#include "Waypoint.h"
#include <Eigen/Dense>

// Default constructor
Waypoint::Waypoint(void){
	point = Eigen::VectorXd::Zero(3);
	Dt = 0;
	special = 0;
}

// Eigen Vector Constructor
Waypoint::Waypoint(const Eigen::Vector3d p, const double t){
	point = p;
	Dt = t;
	special = 0;
}

// Eigen Vector Constructor with special pointer
Waypoint::Waypoint(const Eigen::Vector3d p, const double t, const double sp ){
	point = p;
	Dt = t;
	special = sp;
}

// Individually define each parameter
Waypoint::Waypoint(const double x, const double dx, const double ddx, const double t){
	point = Eigen::Vector3d(x, dx, ddx);
	Dt = t;
	special = 0;
}

// Individually define all parameters
Waypoint::Waypoint(const double x, const double dx, const double ddx, const double t, const double sp){
	point = Eigen::Vector3d(x, dx, ddx);
	Dt = t;
	special = sp;
}

// Specify only a single point. It will assume dx and ddx are 0
Waypoint::Waypoint(const double x, const double t){
	point = Eigen::Vector3d(x, 0, 0);
	Dt = t;
	special = 0;
}

// Specify only a single point with a special parameter. It will assume dx and ddx are 0
Waypoint::Waypoint(const double x, const double t, const double sp){
	point = Eigen::Vector3d(x, 0, 0);
	Dt = t;
	special = sp;
}

Waypoint::~Waypoint(void){}
